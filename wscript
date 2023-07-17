## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('myprotocol', ['internet'])
    module.includes = '.'
    module.source = [
        'model/myprotocol-rtable.cc',
        'model/myprotocol-packet.cc',
        'model/myprotocol-routing-protocol.cc',
        'model/myprotocol-id-cache.cc',
        'helper/myprotocol-helper.cc',
        ]

    # module_test = bld.create_ns3_module_test_library('myprotocol')
    # module_test.source = [
    #     'test/myprotocol-testcase.cc',
    #     ]

    headers = bld(features='ns3header')
    headers.module = 'myprotocol'
    headers.source = [
        'model/myprotocol-rtable.h',
        'model/myprotocol-packet.h',
        'model/myprotocol-routing-protocol.h',
        'model/myprotocol-id-cache.h',
        'helper/myprotocol-helper.h',
        ]
    if (bld.env['ENABLE_EXAMPLES']):
      bld.recurse('examples')

    bld.ns3_python_bindings()
