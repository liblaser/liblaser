#!/usr/bin/env python3
"""Writes the disc images of liblaser's identification test, test_identify.

    make_images.py <output directory>

The images are committed beside this script, which is here to say what they
hold and to make them again: it is not run by the build. They are the least
a reader needs to find its way to the files an identification looks for -
no volume recognition sequence beyond its three descriptors, no integrity
descriptors, no space tables, no timestamps - laid out as the standards say
(ECMA-167 and UDF for the DVDs and the Blu-ray, ECMA-119 for the Video CDs),
by hand and with nothing but the standard library.

    dvd-video.img      UDF 1.02; /VIDEO_TS/VIDEO_TS.IFO, and an empty
                       /AUDIO_TS as many pressed DVD-Videos carry
    dvd-audio.img      UDF 1.02; /AUDIO_TS/AUDIO_TS.IFO
    dvd-universal.img  UDF 1.02; both
    udf-data.img       UDF 1.02; a /VIDEO_TS/VIDEO_TS.IFO that is not one
    bd-video.img       UDF 2.50, its directories in a metadata partition as
                       on a Blu-ray; /BDMV/index.bdmv and three languages of
                       disc library metadata in /BDMV/META/DL
    vcd.iso            ISO 9660; /VCD/INFO.VCD
    svcd-track2.iso    ISO 9660 in a track starting at sector 75;
                       /SVCD/INFO.SVD
"""

import os
import struct
import sys

SECTOR = 2048


# ---- ECMA-167 / UDF ----------------------------------------------------------

def crc_itu_t(data):
    """The descriptor CRC: x^16 + x^12 + x^5 + 1, from 0 (ECMA-167 3/7.2.6)."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def tagged(ident, location, body, version):
    """A descriptor: its 16-byte tag (3/7.2), then @body."""
    tag = bytearray(16)
    struct.pack_into("<HH", tag, 0, ident, version)
    struct.pack_into("<HHI", tag, 8, crc_itu_t(body), len(body), location)
    tag[4] = sum(tag) & 0xFF
    return bytes(tag) + bytes(body)


def dstring(text, size):
    """A dstring: 8-bit characters, its length in the last byte (1/7.2.12)."""
    field = bytearray(size)
    field[0] = 8
    field[1:1 + len(text)] = text.encode("latin-1")
    field[size - 1] = len(text) + 1
    return bytes(field)


def regid(identifier, suffix=b""):
    """An entity identifier (1/7.4)."""
    field = bytearray(32)
    field[1:1 + len(identifier)] = identifier
    field[24:24 + len(suffix)] = suffix
    return bytes(field)


def long_ad(length, block, partition):
    return struct.pack("<IIH6x", length, block, partition)


def short_ad(length, block):
    return struct.pack("<II", length, block)


class Udf:
    """A UDF volume of one partition, starting at sector 272.

    With @metadata, the file set descriptor, the file entries and the
    directories are in a metadata partition (UDF 2.50, 2.2.10): a file of
    the partition, in one extent, that the logical volume maps as a second
    partition. The files' own data stay in the first.
    """

    PARTITION_START = 272
    METADATA_BLOCKS = 32

    def __init__(self, volume_id, revision, metadata=False):
        self.volume_id = volume_id
        self.revision = revision
        self.version = 3 if revision >= 0x0200 else 2
        self.metadata = metadata
        self.blocks = {}                # sector -> its bytes
        # Where the next block of each partition goes. The metadata file's
        # two entries come first in the partition, then its extent.
        self.next_data = 2 + self.METADATA_BLOCKS if metadata else 0
        self.next_meta = 0
        self.meta_start = 2
        # The partition the directories and entries are in, as a reference.
        self.icb_partition = 1 if metadata else 0
        self.fsd_block = self.alloc_icb()
        self.root = self.Directory(self, None)

    # -- blocks --

    def alloc_data(self, count=1):
        block = self.next_data
        self.next_data += count
        return block

    def alloc_icb(self, count=1):
        if not self.metadata:
            return self.alloc_data(count)
        block = self.next_meta
        self.next_meta += count
        assert self.next_meta <= self.METADATA_BLOCKS
        return block

    def put_data(self, block, data):
        self.put_sector(self.PARTITION_START + block, data)

    def put_icb(self, block, data):
        if self.metadata:
            block += self.meta_start
        self.put_sector(self.PARTITION_START + block, data)

    def put_sector(self, sector, data):
        for i in range(0, max(len(data), 1), SECTOR):
            self.blocks[sector + i // SECTOR] = bytes(data[i:i + SECTOR])

    # -- files --

    def file_entry(self, block, file_type, length, allocation, descriptors):
        """A file entry (4/14.9), its allocation descriptors after it."""
        body = bytearray(176 - 16)
        struct.pack_into("<H", body, 20 - 16, 4)        # strategy
        struct.pack_into("<H", body, 24 - 16, 1)        # entries
        body[27 - 16] = file_type
        struct.pack_into("<H", body, 34 - 16, allocation)
        struct.pack_into("<H", body, 48 - 16, 1)        # links
        struct.pack_into("<Q", body, 56 - 16, length)
        struct.pack_into("<Q", body, 64 - 16, (length + SECTOR - 1) // SECTOR)
        struct.pack_into("<II", body, 168 - 16, 0, len(descriptors))
        return tagged(261, block, bytes(body) + descriptors, self.version)

    def extended_file_entry(self, block, file_type, length, descriptors):
        """An extended file entry (4/14.17), which the metadata file's has
        to be (UDF 2.50, 2.2.13.1): the same, with more before the
        allocation descriptors. Short ones, here."""
        body = bytearray(216 - 16)
        struct.pack_into("<H", body, 20 - 16, 4)        # strategy
        struct.pack_into("<H", body, 24 - 16, 1)        # entries
        body[27 - 16] = file_type
        struct.pack_into("<H", body, 48 - 16, 1)        # links
        struct.pack_into("<QQQ", body, 56 - 16, length, length,
                         (length + SECTOR - 1) // SECTOR)
        struct.pack_into("<II", body, 208 - 16, 0, len(descriptors))
        return tagged(266, block, bytes(body) + descriptors, self.version)

    class Directory:
        def __init__(self, udf, parent):
            self.udf = udf
            self.block = udf.alloc_icb()
            self.parent = parent if parent is not None else self
            self.entries = []           # (name, is a directory, entry block)
            self.children = []
            # Recorded in its own entry rather than in a block of its own,
            # as a directory small enough may be.
            self.inline = False

        def directory(self, name):
            child = Udf.Directory(self.udf, self)
            self.entries.append((name, True, child.block))
            self.children.append(child)
            return child

        def file(self, name, data, inline=False):
            """A file: its data in the entry itself, or in the partition."""
            udf = self.udf
            block = udf.alloc_icb()
            if inline:
                entry = udf.file_entry(block, 5, len(data), 3, data)
            else:
                count = max(1, (len(data) + SECTOR - 1) // SECTOR)
                at = udf.alloc_data(count)
                udf.put_data(at, data)
                # On a Blu-ray the entry is in the metadata partition and
                # says, with a long descriptor, that the data is not.
                if udf.metadata:
                    entry = udf.file_entry(block, 5, len(data), 1,
                                           long_ad(len(data), at, 0))
                else:
                    entry = udf.file_entry(block, 5, len(data), 0,
                                           short_ad(len(data), at))
            udf.put_icb(block, entry)
            self.entries.append((name, False, block))

        def identifier(self, at, name, is_directory, block, is_parent=False):
            """A file identifier descriptor (4/14.4)."""
            encoded = b"" if is_parent else b"\x08" + name.encode("latin-1")
            body = bytearray(38 - 16) + encoded
            struct.pack_into("<H", body, 16 - 16, 1)    # version
            body[18 - 16] = (0x02 if is_directory else 0) | \
                            (0x08 if is_parent else 0)
            body[19 - 16] = len(encoded)
            body[20 - 16:36 - 16] = long_ad(SECTOR, block,
                                            self.udf.icb_partition)
            while (16 + len(body)) % 4:
                body.append(0)
            return tagged(257, at, body, self.udf.version)

        def finish(self):
            """The directory itself, once all it holds has been added."""
            udf = self.udf
            inline = self.inline
            for child in self.children:
                child.finish()
            at = self.block if inline else udf.alloc_icb()
            data = self.identifier(at, "", True, self.parent.block, True)
            for name, is_directory, block in self.entries:
                data += self.identifier(at, name, is_directory, block)
            assert len(data) <= SECTOR - 176
            if inline:
                entry = udf.file_entry(self.block, 4, len(data), 3, data)
            else:
                udf.put_icb(at, data)
                if udf.metadata:
                    descriptor, kind = long_ad(len(data), at, 1), 1
                else:
                    descriptor, kind = short_ad(len(data), at), 0
                entry = udf.file_entry(self.block, 4, len(data), kind,
                                       descriptor)
            udf.put_icb(self.block, entry)

    # -- the volume --

    def image(self):
        self.root.finish()
        version = self.version

        # File set descriptor (4/14.1): where the root directory is.
        body = bytearray(512 - 16)
        struct.pack_into("<HH", body, 28 - 16, 3, 3)    # interchange levels
        struct.pack_into("<II", body, 32 - 16, 1, 1)    # character sets
        body[112 - 16:240 - 16] = dstring(self.volume_id, 128)
        body[304 - 16:336 - 16] = dstring(self.volume_id, 32)
        body[400 - 16:416 - 16] = long_ad(SECTOR, self.root.block,
                                          self.icb_partition)
        body[416 - 16:448 - 16] = regid(b"*OSTA UDF Compliant",
                                        struct.pack("<H", self.revision))
        self.put_icb(self.fsd_block, tagged(256, self.fsd_block, body,
                                            version))

        if self.metadata:
            # The metadata file and its mirror (file types 250 and 251):
            # one extent of the partition, which is the metadata partition.
            extent = short_ad(self.METADATA_BLOCKS * SECTOR, self.meta_start)
            for block, file_type in ((0, 250), (1, 251)):
                self.put_data(block, self.extended_file_entry(
                    block, file_type, self.METADATA_BLOCKS * SECTOR, extent))

        partition_blocks = self.next_data

        # Volume recognition sequence (3/9), from sector 16.
        nsr = b"NSR03" if version == 3 else b"NSR02"
        for i, ident in enumerate((b"BEA01", nsr, b"TEA01")):
            self.put_sector(16 + i, b"\x00" + ident + b"\x01")

        # Volume descriptor sequence (3/8.4), from sector 32, and again from
        # sector 48 as the reserve one.
        primary = bytearray(512 - 16)
        struct.pack_into("<II", primary, 16 - 16, 0, 0)
        primary[24 - 16:56 - 16] = dstring(self.volume_id, 32)
        struct.pack_into("<HHHHII", primary, 56 - 16, 1, 1, 2, 2, 1, 1)
        primary[72 - 16:200 - 16] = dstring(self.volume_id, 128)

        partition = bytearray(512 - 16)
        struct.pack_into("<IHH", partition, 16 - 16, 1, 1, 0)
        partition[24 - 16:56 - 16] = regid(b"+" + nsr)
        struct.pack_into("<III", partition, 184 - 16, 1,    # read only
                         self.PARTITION_START, partition_blocks)

        maps = struct.pack("<BBHH", 1, 6, 1, 0)         # the partition
        if self.metadata:
            # The metadata partition (UDF 2.50, 2.2.10): the entries of its
            # file and of the mirror, and no bitmap.
            extra = bytearray(64)
            extra[0], extra[1] = 2, 64
            extra[4:36] = regid(b"*UDF Metadata Partition",
                                struct.pack("<H", self.revision))
            struct.pack_into("<HHIIIIH", extra, 36, 1, 0, 0, 1, 0xFFFFFFFF,
                             32, 32)
            maps += bytes(extra)

        logical = bytearray(440 - 16) + maps
        struct.pack_into("<I", logical, 16 - 16, 2)
        logical[20 - 16:84 - 16] = b"\x00OSTA Compressed Unicode".ljust(64,
                                                                        b"\0")
        logical[84 - 16:212 - 16] = dstring(self.volume_id, 128)
        struct.pack_into("<I", logical, 212 - 16, SECTOR)
        logical[216 - 16:248 - 16] = regid(b"*OSTA UDF Compliant",
                                           struct.pack("<H", self.revision))
        logical[248 - 16:264 - 16] = long_ad(SECTOR, self.fsd_block,
                                             self.icb_partition)
        struct.pack_into("<II", logical, 264 - 16, len(maps),
                         2 if self.metadata else 1)

        for first in (32, 48):
            sequence = ((1, primary), (5, partition), (6, logical),
                        (8, bytearray(512 - 16)))
            for i, (ident, body) in enumerate(sequence):
                self.put_sector(first + i,
                                tagged(ident, first + i, body, version))

        # Anchor volume descriptor pointer (3/10.2), at sector 256.
        anchor = struct.pack("<IIII", 16 * SECTOR, 32, 16 * SECTOR, 48)
        self.put_sector(256, tagged(2, 256, anchor.ljust(512 - 16, b"\0"),
                                    version))

        sectors = self.PARTITION_START + partition_blocks
        out = bytearray(sectors * SECTOR)
        for sector, data in self.blocks.items():
            out[sector * SECTOR:sector * SECTOR + len(data)] = data
        return bytes(out)


def dvd(volume_id, video, audio, empty_audio=False):
    udf = Udf(volume_id, 0x0102)
    if video is not None:
        udf.root.directory("VIDEO_TS").file("VIDEO_TS.IFO",
                                            video.ljust(2 * SECTOR, b"\0"))
    if audio is not None:
        udf.root.directory("AUDIO_TS").file("AUDIO_TS.IFO",
                                            audio.ljust(2 * SECTOR, b"\0"))
    elif empty_audio:
        udf.root.directory("AUDIO_TS")
    return udf.image()


def disc_library(title):
    return ('<?xml version="1.0" encoding="utf-8"?>\n'
            '<disclib xmlns="urn:BDA:bdmv;disclib">\n'
            '  <di:discinfo xmlns:di="urn:BDA:bdmv;discinfo">\n'
            '    <di:title><di:name>%s</di:name></di:title>\n'
            '  </di:discinfo>\n'
            '</disclib>\n' % title).encode("utf-8")


def bluray():
    udf = Udf("LASER_BD", 0x0250, metadata=True)
    bdmv = udf.root.directory("BDMV")
    bdmv.file("index.bdmv", b"INDX0200".ljust(SECTOR + 100, b"\0"))
    library = bdmv.directory("META").directory("DL")
    library.inline = True
    library.file("bdmt_deu.xml", disc_library("Blu-ray"))
    library.file("bdmt_eng.xml", disc_library("Laser test disc"))
    library.file("bdmt_fra.xml",
                 disc_library("Disque d&apos;essai &#233;t&#233;"),
                 inline=True)
    return udf.image()


# ---- ECMA-119 ----------------------------------------------------------------

def both(fmt, value):
    """A number recorded in both byte orders (ECMA-119 7.2.3, 7.3.3)."""
    return struct.pack("<" + fmt, value) + struct.pack(">" + fmt, value)


def directory_record(name, sector, length, is_directory):
    """A directory record (ECMA-119 9.1)."""
    record = bytearray(33) + name
    record[2:10] = both("I", sector)
    record[10:18] = both("I", length)
    record[25] = 0x02 if is_directory else 0
    record[28:32] = both("H", 1)
    record[32] = len(name)
    if len(record) % 2:
        record.append(0)
    record[0] = len(record)
    return bytes(record)


def video_cd(volume_id, directory, info_name, signature, track_start=0):
    """An ISO 9660 volume in a track starting at sector @track_start: its
    descriptors from the track's sector 16, its extents counted from the
    start of the disc, as a multi-session mastering writes them."""
    path_l, path_m, root, sub, info = (track_start + n
                                       for n in (18, 19, 20, 21, 22))
    out = bytearray((track_start + 23) * SECTOR)

    def put(sector, data):
        out[sector * SECTOR:sector * SECTOR + len(data)] = data

    put(root, directory_record(b"\x00", root, SECTOR, True) +
        directory_record(b"\x01", root, SECTOR, True) +
        directory_record(directory, sub, SECTOR, True))
    put(sub, directory_record(b"\x00", sub, SECTOR, True) +
        directory_record(b"\x01", root, SECTOR, True) +
        directory_record(info_name + b";1", info, SECTOR, False))
    put(info, signature.ljust(SECTOR, b"\0"))

    # The path tables (ECMA-119 9.4), in each byte order: the root, then
    # the directory in it. The identification does not read them; a reader
    # that does finds the volume whole.
    def path_table(order):
        table = b""
        for name, sector in ((b"\x00", root), (directory, sub)):
            table += struct.pack(order + "BBIH", len(name), 0, sector, 1)
            table += name + b"\0" * (len(name) % 2)
        return table

    put(path_l, path_table("<"))
    put(path_m, path_table(">"))

    primary = bytearray(SECTOR)
    primary[0:7] = b"\x01CD001\x01"
    primary[8:40] = b" " * 32
    primary[40:72] = volume_id.ljust(32)
    primary[80:88] = both("I", track_start + 23)
    primary[120:124] = both("H", 1)
    primary[124:128] = both("H", 1)
    primary[128:132] = both("H", SECTOR)
    primary[132:140] = both("I", len(path_table("<")))
    primary[140:144] = struct.pack("<I", path_l)
    primary[148:152] = struct.pack(">I", path_m)
    primary[156:190] = directory_record(b"\x00", root, SECTOR, True)
    primary[881] = 1
    put(track_start + 16, primary)
    put(track_start + 17, b"\xffCD001\x01")
    return bytes(out)


# ------------------------------------------------------------------------------

IMAGES = {
    "dvd-video.img": lambda: dvd("LASER_DVD", b"DVDVIDEO-VMG", None,
                                 empty_audio=True),
    "dvd-audio.img": lambda: dvd("LASER_DVDA", None, b"DVDAUDIO-AMG"),
    "dvd-universal.img": lambda: dvd("LASER_BOTH", b"DVDVIDEO-VMG",
                                     b"DVDAUDIO-AMG"),
    "udf-data.img": lambda: dvd("LASER_DATA", b"not a manager", None),
    "bd-video.img": bluray,
    "vcd.iso": lambda: video_cd(b"LASER_VCD", b"VCD", b"INFO.VCD",
                                b"VIDEO_CD"),
    "svcd-track2.iso": lambda: video_cd(b"LASER_SVCD", b"SVCD", b"INFO.SVD",
                                        b"SUPERVCD", track_start=75),
}

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    os.makedirs(sys.argv[1], exist_ok=True)
    for name, make in IMAGES.items():
        data = make()
        with open(os.path.join(sys.argv[1], name), "wb") as image:
            image.write(data)
        print("%-18s %4d sectors" % (name, len(data) // SECTOR))
