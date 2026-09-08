#!/usr/bin/env python3

# flake8: noqa

'''
script to create ap_romfs_embedded.h from a set of static files

Andrew Tridgell
May 2017
'''

import os, sys, zlib, json, hashlib

def write_encode(out, s):
    out.write(s.encode())

def crc32(bytes, crc=0):
    '''crc32 equivalent to crc32_small() from AP_Math/crc.cpp'''
    for byte in bytes:
        crc ^= byte
        for i in range(8):
            mask = (-(crc & 1)) & 0xFFFFFFFF
            crc >>= 1
            crc ^= (0xEDB88320 & mask)
    return crc

def array_name_for(embedded_name):
    '''a C array name that's stable for a given embedded ROMFS name,
    independent of its position amongst the other embedded files - so a
    fragment written by compress_fragment() doesn't need to change just
    because other ROMFS files were added or removed'''
    return 'ap_romfs_' + hashlib.sha1(embedded_name.encode()).hexdigest()[:16]

def compress_fragment(src, embedded_name, uncompressed):
    '''compress a single ROMFS file, returning a dict with everything
    assemble_embedded_h() needs to embed it, without needing to touch
    the file's contents again'''
    try:
        contents = open(src,'rb').read()
    except Exception:
        raise Exception("Failed to embed %s" % src)

    if embedded_name.endswith("bootloader.bin"):
        # round size to a multiple of 32 bytes for bootloader, this ensures
        # it can be flashed on a STM32H7 chip
        blen = len(contents)
        pad = (32 - (blen % 32)) % 32
        if pad != 0:
            contents += bytes([0xff]*pad)
            print("Padded %u bytes for %s to %u" % (pad, embedded_name, len(contents)))

    crc = crc32(contents)

    if uncompressed:
        # terminate if there's not already an existing null. we don't add it to
        # the contents to avoid storing the wrong length
        null_terminate = 0 not in contents
        b = contents
    else:
        # compress it (max level, max window size, raw stream, max mem usage)
        z = zlib.compressobj(level=9, method=zlib.DEFLATED, wbits=-15, memLevel=9)
        b = z.compress(contents)
        b += z.flush()
        # decompressed data will be null terminated at runtime, nothing to do here
        null_terminate = False

    if len(b) == 0:
        raise ValueError(f"Zero-length ROMFS contents ({embedded_name}) not permitted")

    return {
        'name': embedded_name,
        'array_name': array_name_for(embedded_name),
        'crc': crc,
        'decompressed_size': len(contents),
        'body': ",".join(str(c) for c in b),
        'null_terminate': null_terminate,
    }

def write_fragment(out_path, src, embedded_name, uncompressed):
    '''compress one ROMFS file and write its fragment out as JSON, for
    assemble_embedded_h() to pick up later without recompressing it'''
    frag = compress_fragment(src, embedded_name, uncompressed)
    with open(out_path, 'w') as f:
        json.dump(frag, f)

def assemble_embedded_h(filename, fragments, uncompressed=False):
    '''assemble a ap_romfs_embedded.h file from a list of (embedded_name,
    fragment_path) pairs, each already compressed by write_fragment()'''

    # remove duplicates and sort
    fragments = sorted(set(fragments))

    done = set()
    frags = []
    for name, fragment_path in fragments:
        if name in done:
            print("Duplicate ROMFS file %s" % name)
            return False
        done.add(name)
        with open(fragment_path) as f:
            frags.append(json.load(f))

    out = open(filename, "wb")
    write_encode(out, '''// generated embedded files for AP_ROMFS\n\n''')

    for frag in frags:
        write_encode(out, '__EXTFLASHFUNC__ static const uint8_t %s[] = {' % frag['array_name'])
        write_encode(out, frag['body'])
        if frag['null_terminate']:
            write_encode(out, ",0")
        write_encode(out, '};\n\n')

    write_encode(out, '''const AP_ROMFS::embedded_file AP_ROMFS::files[] = {\n''')
    for frag in frags:
        ustr = ' (uncompressed)' if uncompressed else ''
        print("Embedding file %s%s" % (frag['name'], ustr))
        write_encode(out, '{ "%s", sizeof(%s), %d, 0x%08x, %s },\n' % (
            frag['name'], frag['array_name'], frag['decompressed_size'], frag['crc'], frag['array_name']))
    write_encode(out, '};\n')
    out.close()
    return True

if __name__ == '__main__':
    import sys, tempfile
    with tempfile.TemporaryDirectory() as tmpdir:
        flist = []
        for i, f in enumerate(sys.argv[1:]):
            frag_path = os.path.join(tmpdir, '%u.json' % i)
            write_fragment(frag_path, f, f, False)
            flist.append((f, frag_path))
        assemble_embedded_h("/tmp/ap_romfs_embedded.h", flist)
