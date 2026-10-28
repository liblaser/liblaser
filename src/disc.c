/*****************************************************************************
 * disc.c: disc identification - see laser_disc.h for the contract.
 *****************************************************************************
 * Copyright (C) 2026 Authors
 *
 * Authors: Pierre Bogdanovscky
 * Co-authored-by: claude-code:claude-opus-5-5
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/types.h>

#include "laser.h"
#include "laser_disc.h"
#include "laser_internal.h"   /* logging and laser_token_settle(); commands
                                * go through laser.h only */

#include <udfread.h>
#include <blockinput.h>

/* ============================================================================
 * DVD and BD-Video detection, via libudfread
 * ============================================================================ */

struct scsi_block_input
{
    struct udfread_block_input base; /* must be first member */
    int token;

    /* Latched once the medium or the drive has gone, so that the rest of the
     * walk fails at once instead of asking the drive again. A drive that left
     * the bus fails every command instantly, so without this the walk would
     * fire a burst of dead commands. */
    bool gone;
};

static int udf_read_cb(struct udfread_block_input *bi, uint32_t lba,
                       void *buf, uint32_t nblocks, int flags)
{
    (void) flags;
    struct scsi_block_input *sbi = (struct scsi_block_input *)bi;

    if (sbi->gone)
        return -1;

    int ret = laser_read_blocks(sbi->token, lba, (int)nblocks, buf);

    /* The callback has room for two outcomes only. The terminal statuses are
     * acted on here; a short read is a failure, since half a metadata
     * structure is no answer. */
    if (ret == LASER_ERR_MEDIA_GONE || ret == LASER_ERR_NO_DEVICE)
        sbi->gone = true;

    if (ret != (int)nblocks)
        return -1;

    return ret;
}

/** Copy a NUL-terminated UTF-8 string into a laser_disc_t field of
 * @p dst_size bytes, truncating on a sequence boundary rather than on a byte,
 * so that the result is still UTF-8 - the consumer hands it to a Java String.
 * While the first byte cut off is a continuation byte (10xxxxxx), the cut
 * moves left. */
static void copy_utf8(char *dst, size_t dst_size, const char *src)
{
    size_t len = strlen(src);

    if (len > dst_size - 1) {
        len = dst_size - 1;

        while (len > 0 && ((unsigned char)src[len] & 0xC0) == 0x80)
            len--;
    }

    memcpy(dst, src, len);
    dst[len] = '\0';
}

static uint32_t udf_size_cb(struct udfread_block_input *bi)
{
    /* Unknown; optional per blockinput.h. */
    (void) bi;
    return 0;
}

/* What one pass over the UDF filesystem found. */
typedef enum {
    UDF_DISC_NONE = 0,    /* no UDF, or UDF with no known layout on it */
    UDF_DISC_DVD_VIDEO,   /* a Video zone and no Audio zone */
    UDF_DISC_DVD_AUDIO,   /* an Audio zone and no Video zone */
    UDF_DISC_DVD_UNIVERSAL,  /* both zones - a universal disc */
    UDF_DISC_BD,
} udf_disc_t;

#define UDF_ZONE_MAGIC_LEN 12

/** Is this DVD zone really there? The directory is not enough: many pressed
 * DVD-Videos carry an empty /AUDIO_TS. The zone's manager file must open
 * with its identifier, which is what libdvdread checks at open time.
 *
 * @param path  the zone's manager file, absolute on the UDF volume
 * @param magic its identifier, exactly UDF_ZONE_MAGIC_LEN bytes */
static bool udf_zone_present(udfread *udf, const char *path, const char *magic)
{
    UDFFILE *f = udfread_file_open(udf, path);
    if (f == NULL)
        return false;

    char buf[UDF_ZONE_MAGIC_LEN];
    ssize_t got = udfread_file_read(f, buf, sizeof(buf));
    udfread_file_close(f);

    return got == (ssize_t)sizeof(buf)
        && memcmp(buf, magic, sizeof(buf)) == 0;
}

#define UDF_VIDEO_ZONE_IFO   "/VIDEO_TS/VIDEO_TS.IFO"
#define UDF_VIDEO_ZONE_MAGIC "DVDVIDEO-VMG"
#define UDF_AUDIO_ZONE_IFO   "/AUDIO_TS/AUDIO_TS.IFO"
#define UDF_AUDIO_ZONE_MAGIC "DVDAUDIO-AMG"

/* ============================================================================
 * BD-Video title, from the disc library metadata
 * ============================================================================
 * /BDMV/META/DL holds one bdmt_<lang>.xml per language, <lang> being an
 * ISO 639-2 code, and the title is the text of <di:name> inside
 * <di:discinfo>. That directory is not encrypted - AACS covers the streams -
 * so no key is involved. A scanner rather than an XML library: one element is
 * wanted, from a file of a few KB.
 * ============================================================================ */

#define BDMT_DIR      "/BDMV/META/DL"
#define BDMT_NAME_LEN 12            /* "bdmt_" + 3-letter code + ".xml" */
#define BDMT_MAX_SIZE (64 * 1024)   /* bound on the bytes read from one file */
#define BDMT_MAX_FILES 32           /* bound on the languages tried */

/** Rank of a directory entry as a title source, lower is better: the index
 * of its code in @p langs, then "eng", then any other language. -1 for a
 * name that is not a bdmt file. */
static int bdmt_rank(const char *name, const char *const *langs)
{
    if (strlen(name) != BDMT_NAME_LEN || strncasecmp(name, "bdmt_", 5) != 0 ||
        strcasecmp(name + 8, ".xml") != 0)
        return -1;

    const char *code = name + 5;
    int i = 0;

    if (langs != NULL)
        for (; langs[i] != NULL; i++)
            if (strlen(langs[i]) == 3 && strncasecmp(code, langs[i], 3) == 0)
                return i;

    return strncasecmp(code, "eng", 3) == 0 ? i : i + 1;
}

typedef struct
{
    char name[BDMT_NAME_LEN + 1];
    int rank;
} bdmt_file_t;

/** The bdmt files into @p files, best-ranked first; their number, 0 if there
 * is none. */
static int bdmt_list(udfread *udf, const char *const *langs,
                     bdmt_file_t files[BDMT_MAX_FILES])
{
    UDFDIR *dir = udfread_opendir(udf, BDMT_DIR);
    if (dir == NULL)
        return 0;

    int count = 0;
    struct udfread_dirent ent;

    while (count < BDMT_MAX_FILES && udfread_readdir(dir, &ent) != NULL)
    {
        if (ent.d_type == UDF_DT_DIR)
            continue;

        int rank = bdmt_rank(ent.d_name, langs);
        if (rank < 0)
            continue;

        /* Inserted in rank order, after the files of the same rank. */
        int i = count++;
        while (i > 0 && files[i - 1].rank > rank)
        {
            files[i] = files[i - 1];
            i--;
        }
        memcpy(files[i].name, ent.d_name, BDMT_NAME_LEN + 1);
        files[i].rank = rank;
    }

    udfread_closedir(dir);
    return count;
}

/** Is @p name a placeholder rather than a title: what some discs put in the
 * metadata of a language they were not authored for ("Blu-ray")? */
static bool bdmt_title_is_placeholder(const char *name)
{
    static const char *const placeholders[] = {
        "blu-ray", "blu-ray disc", "bluray", "blu ray", "bd", "bd-rom",
        "bdmv", "disc", "title", "untitled",
    };

    for (size_t i = 0; i < sizeof(placeholders) / sizeof(placeholders[0]); i++)
        if (strcasecmp(name, placeholders[i]) == 0)
            return true;
    return false;
}

/** Is [p, p + len) the tag name @p local, whatever its namespace prefix? */
static bool xml_tag_is(const char *p, size_t len, const char *local)
{
    const char *colon = memchr(p, ':', len);
    if (colon != NULL)
    {
        len -= (size_t)(colon + 1 - p);
        p = colon + 1;
    }
    return len == strlen(local) && memcmp(p, local, len) == 0;
}

/** Is @p cp a valid, printable character? */
static bool cp_printable(unsigned long cp)
{
    return cp >= 0x20 && (cp < 0xD800 || cp > 0xDFFF) && cp <= 0x10FFFF;
}

/** Append the printable code point @p cp to @p o as UTF-8, and return the
 * byte after it. */
static char *utf8_put(char *o, unsigned long cp)
{
    if (cp < 0x80)
        *o++ = (char)cp;
    else if (cp < 0x800)
    {
        *o++ = (char)(0xC0 | (cp >> 6));
        *o++ = (char)(0x80 | (cp & 0x3F));
    }
    else if (cp < 0x10000)
    {
        *o++ = (char)(0xE0 | (cp >> 12));
        *o++ = (char)(0x80 | ((cp >> 6) & 0x3F));
        *o++ = (char)(0x80 | (cp & 0x3F));
    }
    else
    {
        *o++ = (char)(0xF0 | (cp >> 18));
        *o++ = (char)(0x80 | ((cp >> 12) & 0x3F));
        *o++ = (char)(0x80 | ((cp >> 6) & 0x3F));
        *o++ = (char)(0x80 | (cp & 0x3F));
    }
    return o;
}

/** Is @p s well-formed UTF-8? A file that is not would reach a Java String
 * through the consumer, which must not be handed one. */
static bool utf8_valid(const unsigned char *s)
{
    while (*s != '\0')
    {
        unsigned char c = *s++;
        int more;
        unsigned long cp;

        if (c < 0x80)
            continue;
        else if ((c & 0xE0) == 0xC0) { more = 1; cp = c & 0x1F; }
        else if ((c & 0xF0) == 0xE0) { more = 2; cp = c & 0x0F; }
        else if ((c & 0xF8) == 0xF0) { more = 3; cp = c & 0x07; }
        else
            return false;

        for (int i = 0; i < more; i++)
        {
            if ((*s & 0xC0) != 0x80)
                return false;
            cp = (cp << 6) | (*s++ & 0x3F);
        }

        static const unsigned long min[] = { 0, 0x80, 0x800, 0x10000 };
        if (cp < min[more] || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF))
            return false;
    }
    return true;
}

/** Decode the character data [p, p + len) into @p dst, which may be @p p
 * itself since decoding never lengthens it: the five predefined entities and
 * numeric references are resolved, and runs of whitespace become one space,
 * trimmed at both ends. A reference to no printable character counts as
 * whitespace; an unknown entity is kept as written. */
static void xml_decode_text(const char *p, size_t len, char *dst)
{
    static const struct { const char *name; char c; } entities[] = {
        { "amp", '&' }, { "lt", '<' }, { "gt", '>' },
        { "quot", '"' }, { "apos", '\'' },
    };
    const char *end = p + len;
    char *o = dst;
    bool space = false;

    while (p < end)
    {
        if (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
        {
            space = o != dst;
            p++;
            continue;
        }

        /* What to write: one byte as it is, or a decoded code point. */
        char c = *p;
        unsigned long cp = 0;
        bool is_cp = false;
        const char *next = p + 1;

        const char *semi = c == '&' ? memchr(p, ';', (size_t)(end - p)) : NULL;
        if (semi != NULL && p[1] == '#')
        {
            bool hex = p[2] == 'x' || p[2] == 'X';
            char *num_end;
            cp = strtoul(p + (hex ? 3 : 2), &num_end, hex ? 16 : 10);
            if (num_end == semi)
            {
                is_cp = true;
                next = semi + 1;
            }
        }
        else if (semi != NULL)
        {
            size_t n = (size_t)(semi - (p + 1));
            for (size_t i = 0; i < sizeof(entities) / sizeof(entities[0]); i++)
                if (strlen(entities[i].name) == n &&
                    memcmp(p + 1, entities[i].name, n) == 0)
                {
                    c = entities[i].c;
                    next = semi + 1;
                    break;
                }
        }

        p = next;
        if (is_cp && !cp_printable(cp))
        {
            space = o != dst;
            continue;
        }

        if (space)
        {
            *o++ = ' ';
            space = false;
        }
        if (is_cp)
            o = utf8_put(o, cp);
        else
            *o++ = c;
    }
    *o = '\0';
}

/** Find the text of the first <name> inside <discinfo>, namespace prefixes
 * ignored, in the NUL-terminated document @p xml. Decoded in place into
 * @p xml itself, which the caller owns; NULL when there is none. */
static char *bdmt_find_name(char *xml)
{
    char *p = xml;
    bool in_discinfo = false;

    while ((p = strchr(p, '<')) != NULL)
    {
        p++;

        if (strncmp(p, "!--", 3) == 0)
        {
            p = strstr(p + 3, "-->");
            if (p == NULL)
                return NULL;
            continue;
        }

        bool closing = *p == '/';
        if (closing)
            p++;

        const char *tag = p;
        while (*p != '\0' && *p != '>' && *p != '/' && *p != ' ' &&
               *p != '\t' && *p != '\r' && *p != '\n')
            p++;
        size_t tag_len = (size_t)(p - tag);

        char *gt = strchr(p, '>');
        if (gt == NULL)
            return NULL;
        bool empty = gt[-1] == '/';

        if (xml_tag_is(tag, tag_len, "discinfo"))
            in_discinfo = !closing && !empty;
        else if (in_discinfo && !closing && !empty &&
                 xml_tag_is(tag, tag_len, "name"))
        {
            char *text = gt + 1;
            char *text_end = strchr(text, '<');
            if (text_end == NULL)
                return NULL;
            xml_decode_text(text, (size_t)(text_end - text), text);
            return text;
        }

        p = gt + 1;
    }
    return NULL;
}

/** The title in the bdmt file at @p path into @p title (LASER_DISC_TITLE_MAX
 * bytes); false, @p title untouched, when it has none worth showing. */
static bool bdmt_read_file(udfread *udf, int token, const char *path,
                           char *title)
{
    UDFFILE *f = udfread_file_open(udf, path);
    if (f == NULL)
    {
        LOGI("token=%d: %s: cannot open", token, path);
        return false;
    }

    int64_t size = udfread_file_size(f);
    if (size <= 0 || size > BDMT_MAX_SIZE)
        size = BDMT_MAX_SIZE;

    char *xml = malloc((size_t)size + 1);
    ssize_t got = -1;
    if (xml != NULL)
        got = udfread_file_read(f, xml, (size_t)size);
    udfread_file_close(f);

    if (got <= 0)
    {
        LOGI("token=%d: %s: cannot read", token, path);
        free(xml);
        return false;
    }
    xml[got] = '\0';

    bool found = false;
    const char *name = bdmt_find_name(xml);
    if (name == NULL || name[0] == '\0' ||
        !utf8_valid((const unsigned char *)name))
        LOGI("token=%d: %s: no usable title", token, path);
    else if (bdmt_title_is_placeholder(name))
        LOGI("token=%d: %s: placeholder title \"%s\", skipped", token, path,
             name);
    else
    {
        copy_utf8(title, LASER_DISC_TITLE_MAX, name);
        LOGI("token=%d: %s: title \"%s\"", token, path, title);
        found = true;
    }

    free(xml);
    return found;
}

/** Read the BD-Video title into @p title (LASER_DISC_TITLE_MAX bytes): from
 * the best-ranked language whose file has a real title, left empty when none
 * has - the caller falls back to the volume label. */
static void bdmt_read_title(udfread *udf, int token, const char *const *langs,
                            char *title)
{
    bdmt_file_t files[BDMT_MAX_FILES];
    int count = bdmt_list(udf, langs, files);

    if (count == 0)
    {
        LOGI("token=%d: no disc library metadata in %s", token, BDMT_DIR);
        return;
    }

    for (int i = 0; i < count; i++)
    {
        char path[sizeof(BDMT_DIR) + BDMT_NAME_LEN + 1];
        snprintf(path, sizeof(path), BDMT_DIR "/%s", files[i].name);
        if (bdmt_read_file(udf, token, path, title))
            return;
    }
}

/** Mounts the medium's UDF filesystem once and answers every UDF question
 * from it, filling volume_id (LASER_DISC_VOLUME_ID_MAX bytes) when something
 * was recognised, and title (LASER_DISC_TITLE_MAX bytes) for a BD-Video. Mounting is the expensive part; each probe after it is a
 * path lookup, plus a 12-byte read where the lookup hits.
 *
 * Order: the Video zone first, as the commonest. If present, the Audio zone
 * too, since a universal disc has both. If absent, BD-Video before the Audio
 * zone, so that a Blu-ray pays no lookup for a zone it cannot have. */
static udf_disc_t detect_udf_disc(int token, const char *const *langs,
                                  char *volume_id, char *title)
{
    udf_disc_t kind = UDF_DISC_NONE;

    struct scsi_block_input sbi = {
            .base = { .close = NULL, .read = udf_read_cb, .size = udf_size_cb },
            .token = token,
            .gone = false,
    };

    udfread *udf = udfread_init();
    if (udf == NULL)
    {
        LOGW("token=%d: udfread_init() failed", token);
        return UDF_DISC_NONE;
    }

    int udf_ret = udfread_open_input(udf, &sbi.base);
    if (udf_ret != 0)
    {
        LOGI("token=%d: no UDF volume (udfread_open_input returned %d, "
             "medium %s)", token, udf_ret, sbi.gone ? "gone" : "present");
    }
    else
    {
        bool has_video = udf_zone_present(udf, UDF_VIDEO_ZONE_IFO,
                                          UDF_VIDEO_ZONE_MAGIC);
        LOGI("token=%d: UDF volume opened, %s: %s", token,
             UDF_VIDEO_ZONE_IFO, has_video ? "present" : "absent");

        if (has_video)
        {
            bool has_audio = udf_zone_present(udf, UDF_AUDIO_ZONE_IFO,
                                              UDF_AUDIO_ZONE_MAGIC);
            LOGI("token=%d: %s: %s", token, UDF_AUDIO_ZONE_IFO,
                 has_audio ? "present" : "absent");
            kind = has_audio ? UDF_DISC_DVD_UNIVERSAL : UDF_DISC_DVD_VIDEO;
        }
        else
        {
            UDFFILE *f = udfread_file_open(udf, "/BDMV/index.bdmv");
            LOGI("token=%d: /BDMV/index.bdmv: %s", token,
                 f != NULL ? "present" : "absent");
            if (f != NULL)
            {
                kind = UDF_DISC_BD;
                udfread_file_close(f);
                bdmt_read_title(udf, token, langs, title);
            }
            else
            {
                bool has_audio = udf_zone_present(udf, UDF_AUDIO_ZONE_IFO,
                                                  UDF_AUDIO_ZONE_MAGIC);
                LOGI("token=%d: %s: %s", token, UDF_AUDIO_ZONE_IFO,
                     has_audio ? "present" : "absent");
                if (has_audio)
                    kind = UDF_DISC_DVD_AUDIO;
            }
        }

        if (kind != UDF_DISC_NONE)
        {
            const char *vol_id = udfread_get_volume_id(udf);
            if (vol_id != NULL)
                copy_utf8(volume_id, LASER_DISC_VOLUME_ID_MAX, vol_id);
            else
                volume_id[0] = '\0';
        }
    }

    udfread_close(udf);
    return kind;
}

/* ============================================================================
 * Video CD and Super Video CD detection, via a minimal ISO9660 walk
 * ============================================================================
 * A Video CD carries ISO9660 and no UDF, and VLC's vcd module reads it
 * through ISO9660. Only two directory levels are needed - the root, then /VCD
 * or /SVCD - so this walks exactly that: no Joliet, no Rock Ridge, no path
 * table, no recursion.
 *
 * The volume descriptors start at sector 16 of the track carrying the
 * filesystem; the extents they lead to are absolute LBAs on the disc. That
 * is how multi-session mastering tools write them and how Linux's isofs
 * reads them, so the track's start offsets the PVD alone.
 * ============================================================================ */

#define ISO_SECTOR_SIZE 2048
#define ISO_PVD_LBA     16

/* Ceiling on the sectors of one directory walked, an order of magnitude above
 * what a VCD needs: a corrupt length field must not turn the probe into an
 * unbounded run of commands. */
#define ISO_MAX_DIR_SECTORS 4

/* Offsets within a directory record (ECMA-119 section 9.1). */
#define ISO_DR_LENGTH        0
#define ISO_DR_EXTENT_LE     2
#define ISO_DR_DATA_LEN_LE  10
#define ISO_DR_FILE_FLAGS   25
#define ISO_DR_NAME_LEN     32
#define ISO_DR_NAME         33

#define ISO_FLAG_DIRECTORY  0x02

/** The token to read through, and the start LBA of the track carrying the
 * filesystem, where its PVD is found - from the TOC read_cd_toc() already
 * fetched, at no extra command. Extents are absolute and not offset by it. */
typedef struct {
    int      token;
    uint32_t base;
} iso_ctx_t;

static uint32_t iso_le32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/** One 2048-byte sector, or false. Every failure is the same here: the probe
 * stops and reports nothing. */
static bool iso_read_sector(const iso_ctx_t *ctx, uint32_t lba, uint8_t *buf)
{
    return laser_read_blocks(ctx->token, lba, 1, buf) == 1;
}

/** Compare a directory record's name against a plain ASCII one, ignoring
 * case and the ";1" version suffix of file identifiers - many discs ignore
 * the uppercase rule. */
static bool iso_name_matches(const uint8_t *name, unsigned name_len,
                             const char *want)
{
    const uint8_t *semi = memchr(name, ';', name_len);
    if (semi != NULL)
        name_len = (unsigned)(semi - name);

    if (name_len != strlen(want))
        return false;

    for (unsigned i = 0; i < name_len; i++)
    {
        int a = name[i];
        int b = (unsigned char)want[i];

        if (a >= 'a' && a <= 'z')
            a -= 'a' - 'A';
        if (b >= 'a' && b <= 'z')
            b -= 'a' - 'A';

        if (a != b)
            return false;
    }

    return true;
}

/** Find one named child of the directory at (dir_lba, dir_len), filling
 * *out_lba and *out_len with its extent.
 *
 * @param want_dir  whether the wanted entry is a directory, checked so that
 *                  a FILE named VCD is never walked as directory records.
 */
static bool iso_find_child(const iso_ctx_t *ctx, uint32_t dir_lba, uint32_t dir_len,
                           const char *want, bool want_dir,
                           uint32_t *out_lba, uint32_t *out_len)
{
    uint32_t sectors = (dir_len + ISO_SECTOR_SIZE - 1) / ISO_SECTOR_SIZE;
    if (sectors > ISO_MAX_DIR_SECTORS)
        sectors = ISO_MAX_DIR_SECTORS;

    for (uint32_t s = 0; s < sectors; s++)
    {
        uint8_t sector[ISO_SECTOR_SIZE];
        if (!iso_read_sector(ctx, dir_lba + s, sector))
            return false;

        unsigned off = 0;
        while (off < ISO_SECTOR_SIZE)
        {
            unsigned rec_len = sector[off + ISO_DR_LENGTH];

            /* Records never straddle sectors: zero is the padding after the
             * last one, ending this sector but not the directory. */
            if (rec_len == 0)
                break;

            /* Malformed: too short for the fixed part, or running past the
             * sector. */
            if (rec_len < ISO_DR_NAME || off + rec_len > ISO_SECTOR_SIZE)
                break;

            unsigned name_len = sector[off + ISO_DR_NAME_LEN];
            if (name_len == 0 || ISO_DR_NAME + name_len > rec_len)
            {
                off += rec_len;
                continue;
            }

            bool is_dir =
                (sector[off + ISO_DR_FILE_FLAGS] & ISO_FLAG_DIRECTORY) != 0;

            if (is_dir == want_dir &&
                iso_name_matches(sector + off + ISO_DR_NAME, name_len, want))
            {
                *out_lba = iso_le32(sector + off + ISO_DR_EXTENT_LE);
                *out_len = iso_le32(sector + off + ISO_DR_DATA_LEN_LE);
                return true;
            }

            off += rec_len;
        }
    }

    return false;
}

/** Does the info file at this extent open with the expected signature? As
 * for a DVD zone, the directory alone is not the answer; these eight bytes
 * are what VLC's vcd module checks. */
static bool iso_info_magic_is(const iso_ctx_t *ctx, uint32_t lba, const char *magic)
{
    uint8_t sector[ISO_SECTOR_SIZE];
    if (!iso_read_sector(ctx, lba, sector))
        return false;

    return memcmp(sector, magic, 8) == 0;
}

typedef enum {
    ISO_VIDEO_NONE = 0,
    ISO_VIDEO_VCD,
    ISO_VIDEO_SVCD,
} iso_video_t;

/** Is this Volume Identifier printable ASCII? ECMA-119 allows only
 * d-characters, but many discs carry bytes in an unrecorded encoding, which
 * would reach the consumer as invalid UTF-8. Such a label is refused whole
 * rather than guessed at; control bytes too, an embedded NUL included. */
static bool iso_volume_id_is_printable_ascii(const uint8_t *p, unsigned len)
{
    for (unsigned i = 0; i < len; i++)
    {
        if (p[i] < 0x20 || p[i] > 0x7E)
            return false;
    }

    return true;
}

/** Walks the ISO9660 filesystem once for both Video CD kinds, filling
 * volume_id (LASER_DISC_VOLUME_ID_MAX bytes) when something was recognised.
 * VCD is tried first only because it is the commoner; a disc cannot be
 * both. */
static iso_video_t detect_iso_video(const iso_ctx_t *ctx, char *volume_id)
{
    uint8_t pvd[ISO_SECTOR_SIZE];
    if (!iso_read_sector(ctx, ctx->base + ISO_PVD_LBA, pvd))
        return ISO_VIDEO_NONE;

    /* Type 1, the Primary Volume Descriptor, and the "CD001" identifier:
     * either alone matches too much. */
    if (pvd[0] != 0x01 || memcmp(pvd + 1, "CD001", 5) != 0)
        return ISO_VIDEO_NONE;

    /* The root directory record, at offset 156 of the PVD. */
    const uint8_t *root = pvd + 156;
    uint32_t root_lba = iso_le32(root + ISO_DR_EXTENT_LE);
    uint32_t root_len = iso_le32(root + ISO_DR_DATA_LEN_LE);

    if (root_len == 0)
        return ISO_VIDEO_NONE;

    static const struct {
        const char *dir;
        const char *info;
        const char *magic;
        iso_video_t kind;
    } probes[] = {
        { "VCD",  "INFO.VCD", "VIDEO_CD", ISO_VIDEO_VCD  },
        { "SVCD", "INFO.SVD", "SUPERVCD", ISO_VIDEO_SVCD },
    };

    iso_video_t kind = ISO_VIDEO_NONE;

    for (size_t i = 0; i < sizeof(probes) / sizeof(probes[0]); i++)
    {
        uint32_t dir_lba, dir_len, info_lba, info_len;

        if (!iso_find_child(ctx, root_lba, root_len, probes[i].dir, true,
                            &dir_lba, &dir_len))
            continue;

        if (!iso_find_child(ctx, dir_lba, dir_len, probes[i].info, false,
                            &info_lba, &info_len))
            continue;

        if (info_len < 8)
            continue;

        if (iso_info_magic_is(ctx, info_lba, probes[i].magic))
        {
            kind = probes[i].kind;
            break;
        }
    }

    if (kind == ISO_VIDEO_NONE)
        return ISO_VIDEO_NONE;

    /* Volume Identifier: 32 bytes at offset 40, padded with spaces rather
     * than terminated. 32 bytes fit LASER_DISC_VOLUME_ID_MAX with its
     * terminator. */
    unsigned len = 32;
    while (len > 0 && (pvd[40 + len - 1] == ' ' || pvd[40 + len - 1] == '\0'))
        len--;

    if (!iso_volume_id_is_printable_ascii(pvd + 40, len))
    {
        /* The disc is still identified; only the label is lost. */
        volume_id[0] = '\0';
        return kind;
    }

    memcpy(volume_id, pvd + 40, len);
    volume_id[len] = '\0';

    return kind;
}

/* ============================================================================
 * What the medium is: GET CONFIGURATION, then READ TOC on a CD
 * ============================================================================ */

/** The medium's family, from the drive's current profile. */
typedef enum {
    /** The drive did not answer, reported no current profile, or reported
     * one this file does not classify. */
    MEDIUM_UNKNOWN = 0,
    MEDIUM_CD,
    MEDIUM_DVD,
    MEDIUM_BD,
} medium_t;

/* Ask for the current profile (MMC GET CONFIGURATION, feature header bytes
 * 6-7). This is what tells a CD from a DVD or a BD: READ TOC cannot, since
 * drives answer it on every medium, making a single-track TOC up for a DVD
 * or a BD. Pre-MMC-3 drives may not implement the command, hence
 * MEDIUM_UNKNOWN as a fallback rather than a verdict. */
static medium_t read_medium(int token)
{
    uint8_t cdb[10] = { 0 };
    cdb[0] = 0x46; /* GET CONFIGURATION */
    cdb[1] = 0x00; /* RT 00b: every feature, of which only the header is read */
    cdb[8] = 32;   /* allocation length */

    uint8_t buf[32] = { 0 };
    int actual_len = 0;
    laser_status_t st = laser_scsi_cdb(token, cdb, sizeof(cdb),
                                       buf, sizeof(buf), 1, &actual_len);
    if (st != LASER_OK || actual_len < 8)
    {
        LOGI("token=%d: GET CONFIGURATION unanswered (status %d, %d bytes) - "
             "medium unknown", token, (int)st, actual_len);
        return MEDIUM_UNKNOWN;
    }

    unsigned profile = ((unsigned)buf[6] << 8) | buf[7];
    medium_t medium;

    if (profile >= 0x0008 && profile <= 0x000A)      /* CD-ROM, CD-R, CD-RW */
        medium = MEDIUM_CD;
    else if (profile >= 0x0010 && profile <= 0x002F) /* DVD-ROM ... DVD+R DL */
        medium = MEDIUM_DVD;
    else if (profile >= 0x0040 && profile <= 0x004F) /* BD-ROM, BD-R, BD-RE */
        medium = MEDIUM_BD;
    else
        medium = MEDIUM_UNKNOWN;

    LOGI("token=%d: current profile 0x%04x (%s)", token, profile,
         medium == MEDIUM_CD  ? "CD"
         : medium == MEDIUM_DVD ? "DVD"
         : medium == MEDIUM_BD  ? "BD" : "unknown");
    return medium;
}

/** What READ TOC says about the first track. */
typedef enum {
    /** READ TOC failed, or returned no usable track entry. */
    CD_TOC_NONE = 0,
    /** The first track is a data track. */
    CD_TOC_DATA,
    /** The first track is an audio track. */
    CD_TOC_AUDIO,
} cd_toc_t;

/* Read the medium's table of contents. Meaningful on a CD only: see
 * read_medium() for what a drive answers on anything else.
 *
 * @param first_data_lba receives the start LBA of the first DATA track, or 0
 *        if there is none - where the ISO9660 walk looks for the PVD.
 */
static cd_toc_t read_cd_toc(int token, uint32_t *first_data_lba)
{
    *first_data_lba = 0;

    uint8_t cdb[10] = { 0 };
    cdb[0] = 0x43; /* READ TOC/PMA/ATIP */
    cdb[1] = 0x00; /* MSF = 0 (LBA addressing) */
    cdb[2] = 0x00; /* format 0: normal TOC */
    cdb[6] = 0x00; /* starting track number */
    cdb[7] = 0x00;
    cdb[8] = 0x80; /* allocation length: 128 bytes - header + a few tracks */

    /* Zeroed, so that bytes a padding bridge counts but never filled read as
     * 0 (see the residue discussion in bot.c). */
    uint8_t toc[128] = { 0 };
    int actual_len = 0;
    laser_status_t st = laser_scsi_cdb(token, cdb, sizeof(cdb),
                                       toc, sizeof(toc), 1, &actual_len);
    if (st != LASER_OK || actual_len < 4 + 8)
    {
        LOGI("token=%d: READ TOC unanswered (status %d, %d bytes)",
             token, (int)st, actual_len);
        return CD_TOC_NONE;
    }

    /* The TOC's own 16-bit length field, which counts the bytes after it, is
     * checked as well as the transfer length: a padded transfer and a bridge
     * with a broken residue look alike to the transport. One full descriptor
     * needs 2 + 2 + 8 bytes. */
    unsigned toc_data_len = ((unsigned)toc[0] << 8) | toc[1];
    if (toc_data_len + 2u < 4u + 8u)
        return CD_TOC_NONE;

    /* Whole descriptors, bounded by the announced length, the transfer and
     * the buffer alike. */
    unsigned avail = toc_data_len + 2u;
    if (avail > (unsigned)actual_len)
        avail = (unsigned)actual_len;
    if (avail > sizeof(toc))
        avail = sizeof(toc);
    unsigned descriptors = (avail - 4u) / 8u;

    /* Descriptor layout: [reserved][ADR/CONTROL][track#][reserved]
     * [start address, 4 bytes, big-endian LBA since MSF=0]. CONTROL is the
     * low nibble of byte 1; bit 0x04 set means a data track, clear audio. */
    for (unsigned i = 0; i < descriptors; i++) {
        const uint8_t *d = toc + 4 + i * 8;

        if ((d[1] & 0x04) == 0)
            continue;                       /* audio track */
        if (d[2] == 0xAA)
            continue;                       /* lead-out, not a real track */

        *first_data_lba = ((uint32_t)d[4] << 24) | ((uint32_t)d[5] << 16) |
                          ((uint32_t)d[6] << 8)  |  (uint32_t)d[7];
        break;
    }

    /* Only the first track decides audio versus data. An Enhanced CD - audio
     * first, data session last - is an audio CD, which is the useful answer.
     * A mixed-mode disc - data first, audio after - is a data disc: rare,
     * and with no obvious right answer. */
    uint8_t control = toc[5] & 0x0F;
    return (control & 0x04) == 0 ? CD_TOC_AUDIO : CD_TOC_DATA;
}


/* ============================================================================
 * Public entry point
 * ============================================================================ */

/* For the final log line: a name rather than a number to look up. */
static const char *laser_disc_kind_name(laser_disc_kind_t kind)
{
    switch (kind)
    {
        case LASER_DISC_CD_AUDIO:      return "audio CD";
        case LASER_DISC_DVD_VIDEO:     return "DVD-Video";
        case LASER_DISC_DVD_AUDIO:     return "DVD-Audio";
        case LASER_DISC_DVD_UNIVERSAL: return "universal DVD";
        case LASER_DISC_BD_VIDEO:      return "BD-Video";
        case LASER_DISC_VCD:           return "Video CD";
        case LASER_DISC_SVCD:          return "Super Video CD";
        case LASER_DISC_UNKNOWN:       break;
    }
    return "unknown";
}

/* Each detector writes out->volume_id only when it recognises something, so
 * a miss leaves the empty label for the next one. */
static bool identify_udf(int token, const char *const *langs,
                         laser_disc_t *out)
{
    switch (detect_udf_disc(token, langs, out->volume_id, out->title))
    {
        case UDF_DISC_DVD_VIDEO:
            out->kind = LASER_DISC_DVD_VIDEO;
            return true;

        case UDF_DISC_DVD_AUDIO:
            out->kind = LASER_DISC_DVD_AUDIO;
            return true;

        case UDF_DISC_DVD_UNIVERSAL:
            out->kind = LASER_DISC_DVD_UNIVERSAL;
            return true;

        case UDF_DISC_BD:
            out->kind = LASER_DISC_BD_VIDEO;
            return true;

        case UDF_DISC_NONE:
            break;
    }
    return false;
}

static bool identify_iso(int token, uint32_t track_lba, laser_disc_t *out)
{
    const iso_ctx_t ctx = { .token = token, .base = track_lba };

    switch (detect_iso_video(&ctx, out->volume_id))
    {
        case ISO_VIDEO_VCD:
            out->kind = LASER_DISC_VCD;
            return true;

        case ISO_VIDEO_SVCD:
            out->kind = LASER_DISC_SVCD;
            return true;

        case ISO_VIDEO_NONE:
            break;
    }
    return false;
}

void laser_disc_identify_lang(int token, const char *const *langs,
                              laser_disc_t *out)
{
    /* Zeroed first, so that every path that recognises nothing leaves
     * LASER_DISC_UNKNOWN and an empty label. */
    memset(out, 0, sizeof(*out));

    /* The disc may have been swapped since the device was registered. A
     * drive whose readiness wait already ran out is only checked once more,
     * not waited for again: ready now, it is probed. */
    laser_token_settle(token);

    /* A drive still not ready would answer every probe below the same way,
     * each after a full retry budget. */
    if (laser_token_not_ready(token)) {
        LOGI("token=%d: drive did not become ready, not probing the disc",
             token);
        return;
    }

    const medium_t medium = read_medium(token);

    /* READ TOC only where its answer means something: on a CD, or on a
     * medium the drive would not name. */
    uint32_t track_lba = 0;
    cd_toc_t toc = CD_TOC_NONE;

    if (medium == MEDIUM_CD || medium == MEDIUM_UNKNOWN)
    {
        toc = read_cd_toc(token, &track_lba);
        LOGI("token=%d: READ TOC says %s (first data track at LBA %u)", token,
             toc == CD_TOC_AUDIO ? "audio first track"
             : toc == CD_TOC_DATA ? "data first track" : "nothing usable",
             track_lba);

        if (toc == CD_TOC_AUDIO)
        {
            out->kind = LASER_DISC_CD_AUDIO;
            /* volume_id stays empty: a Red Book disc has no filesystem. */
            goto done;
        }
    }

    /* On a known CD, the ISO9660 walk first: a Video CD is the likeliest
     * video disc there, and the walk is cheaper than a UDF mount. UDF still
     * follows, for a DVD-Video burned on a CD. On an unknown medium the
     * order is reversed, DVDs being likelier. */
    if (medium == MEDIUM_CD && toc == CD_TOC_DATA &&
        identify_iso(token, track_lba, out))
        goto done;

    if (identify_udf(token, langs, out))
        goto done;

    if (medium == MEDIUM_UNKNOWN && toc == CD_TOC_DATA)
        identify_iso(token, track_lba, out);

done:
    LOGI("token=%d: identified as %s, volume id \"%s\", title \"%s\"", token,
         laser_disc_kind_name(out->kind), out->volume_id, out->title);
}

void laser_disc_identify(int token, laser_disc_t *out)
{
    laser_disc_identify_lang(token, NULL, out);
}
