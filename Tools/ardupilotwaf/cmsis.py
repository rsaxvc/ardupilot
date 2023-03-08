# encoding: utf-8

"""
Waf tool for building CMSIS-DSP
"""

from waflib import Errors, Logs, Task, Utils, Context
from waflib.TaskGen import after_method, before_method, feature

import os
import sys
import re

_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/ChibiOS')
    include_dirs_node = bldnode.find_node('include_dirs')
    if include_dirs_node is None:
        _dynamic_env_data['include_dirs'] = []
        return
    tmp_str = include_dirs_node.read()
    tmp_str = tmp_str.replace(';\n','')
    tmp_str = tmp_str.replace('-I','')  #remove existing -I flags
    # split, coping with separator
    idirs = re.split('; ', tmp_str)

    # create unique list, coping with relative paths
    idirs2 = []
    for d in idirs:
        if d.startswith('../'):
            # relative paths from the make build are relative to BUILDROOT
            d = os.path.join(bld.env.BUILDROOT, d)
        d = os.path.normpath(d)
        if d not in idirs2:
            idirs2.append(d)
    _dynamic_env_data['include_dirs'] = idirs2

@feature('ch_ap_library', 'ch_ap_program')
@before_method('process_source')
def ch_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)
    self.use += ' ch'
    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])


@feature('ch_ap_program')
@after_method('process_source')
def chibios_firmware(self):
    yield


def setup_cmsis_build(cfg):
    '''enable CMSIS_DSP build. By doing this here we can auto-enable CMSIS_DSP'''
    env = cfg.env

    dirs = [
        'BasicMathFunctions',
        'BayesFunctions',
        'CommonTables',
        'ComplexMathFunctions',
        'ControllerFunctions',
        'DistanceFunctions',
        'FilteringFunctions',
        'StatisticsFunctions',
        'WindowFunctions',
        'QuaternionMathFunctions',
        'InterpolationFunctions',
        'SVMFunctions',
        'TransformFunctions',
        'FastMathFunctions',
        ]

    types = [
        'f32',
        'f64',
        'q15',
        'q32',
        ]

    for dir in dirs:
        for type in types:
            env.AP_LIBRARIES += [
                'AP_GyroFFT',
                'modules/CMSIS_DSP/Source/' + dir + '/arm_*' + type + '.c',
                ]

    env.AP_LIBRARIES += [
        'AP_GyroFFT',
        'modules/CMSIS_DSP/Source/TransformFunctions/arm*bitreversal2.c',
        'modules/CMSIS_DSP/Source/CommonTables/arm*common_tables.c',
        'modules/CMSIS_DSP/Source/CommonTables/arm*const_structs.c',
        ]

    # Do not link in all of the twiddle tables by default.
    # By being selective we save 70k in text space.
    env.DEFINES += [
        'ARM_MATH_DSP',
        'ARM_MATH_LOOPUNROLL',
        'ARM_FAST_ALLOW_TABLES',
        'ARM_TABLE_RECIP_F32',
        'ARM_TABLE_RECIP_Q15',
        'ARM_TABLE_SIN_F32',
        'ARM_TABLE_SIN_Q31',
        'ARM_TABLE_SIN_Q15',
        'ARM_TABLE_SQRT_Q31',
        'ARM_TABLE_SQRT_Q15',
        'ARM_TABLE_TWIDDLECOEF_F32_32',
        'ARM_TABLE_TWIDDLECOEF_F32_64',
        'ARM_TABLE_TWIDDLECOEF_F32_128',
        'ARM_TABLE_TWIDDLECOEF_F32_256',
        ]

    if env.CORTEX != 'cortex-m4':
        env.DEFINES += [
            'ARM_TABLE_TWIDDLECOEF_F32_512',
            'ARM_TABLE_TWIDDLECOEF_F32_1024',
            ]

    env.INCLUDES += [
        cfg.srcnode.find_dir('modules/CMSIS_DSP/Include').abspath(),
        cfg.srcnode.find_dir('modules/CMSIS_DSP/PrivateInclude').abspath(),
        ]
    cfg.get_board().with_cmsis = True

def configure(cfg):
    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['ch_ap_program']

    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['ch_ap_library']

    env.CMSIS_ROOT = srcpath('modules/CMSIS_DSP')
    env.BUILDDIR = bldpath('modules/CMSIS_DSP')
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')

    # relative paths to pass to make, relative to directory that make is run from
    env.CH_ROOT_REL = os.path.relpath(env.CH_ROOT, env.BUILDROOT)
    env.CC_ROOT_REL = os.path.relpath(env.CC_ROOT, env.BUILDROOT)
    env.AP_HAL_REL = os.path.relpath(env.AP_HAL_ROOT, env.BUILDROOT)
    env.BUILDDIR_REL = os.path.relpath(env.BUILDDIR, env.BUILDROOT)

    if cfg.options.default_parameters:
        cfg.msg('Default parameters', cfg.options.default_parameters, color='YELLOW')
        env.DEFAULT_PARAMETERS = cfg.options.default_parameters

    try:
        ret = generate_hwdef_h(env)
    except Exception:
        cfg.fatal("Failed to process hwdef.dat")
    if ret != 0:
        cfg.fatal("Failed to process hwdef.dat ret=%d" % ret)
    load_env_vars(cfg.env)
    if env.HAL_NUM_CAN_IFACES and env.AP_PERIPH and not env.BOOTLOADER:
        setup_canperiph_build(cfg)
    if env.HAL_NUM_CAN_IFACES and env.AP_PERIPH and int(env.HAL_NUM_CAN_IFACES)>1 and not env.BOOTLOADER:
        env.DEFINES += [ 'CANARD_MULTI_IFACE=1' ]
    setup_cmsis_build(cfg)
    setup_optimization(cfg.env)


def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    load_env_vars(bld.env)
    if bld.env.HAL_NUM_CAN_IFACES:
        bld.get_board().with_can = True
    hwdef_h = os.path.join(bld.env.BUILDROOT, 'hwdef.h')
    if not os.path.exists(hwdef_h):
        print("Generating hwdef.h")
        try:
            ret = generate_hwdef_h(bld.env)
        except Exception:
            bld.fatal("Failed to process hwdef.dat")
        if ret != 0:
            bld.fatal("Failed to process hwdef.dat ret=%d" % ret)
    setup_optimization(bld.env)

def build(bld):
    hwdef_rule="%s '%s/hwdef/scripts/chibios_hwdef.py' -D '%s' --params '%s' '%s'" % (
            bld.env.get_flat('PYTHON'),
            bld.env.AP_HAL_ROOT,
            bld.env.BUILDROOT,
            bld.env.default_parameters,
            bld.env.HWDEF)
    if bld.env.HWDEF_EXTRA:
        hwdef_rule += " " + bld.env.HWDEF_EXTRA
    if bld.env.BOOTLOADER_OPTION:
        hwdef_rule += " " + bld.env.BOOTLOADER_OPTION
    
    common_src = [bld.bldnode.find_or_declare('hwdef.h'),
                  bld.bldnode.find_or_declare('hw.dat'),
                  bld.bldnode.find_or_declare('ldscript.ld'),
                  bld.bldnode.find_or_declare('common.ld'),
                  bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')]
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.[ch]')
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.mk')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.[ch]')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.mk')

    if bld.env.ENABLE_CRASHDUMP:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' BUILDROOT='${BUILDROOT}' CRASHCATCHER='${CC_ROOT_REL}' CHIBIOS='${CH_ROOT_REL}' AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} '${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
            group='dynamic_sources',
            source=common_src,
            target=[bld.bldnode.find_or_declare('modules/ChibiOS/libch.a'), bld.bldnode.find_or_declare('modules/ChibiOS/libcc.a')]
        )
    else:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' BUILDROOT='${BUILDROOT}' CHIBIOS='${CH_ROOT_REL}' AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} '${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
            group='dynamic_sources',
            source=common_src,
            target=bld.bldnode.find_or_declare('modules/ChibiOS/libch.a')
        )
    ch_task.name = "CMSIS_lib"
    bld.env.LIB += ['cmsis']
    bld.env.LIBPATH += ['modules/CMSIS_DSP']
