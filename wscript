## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):
    module = bld.create_ns3_module('myprotocol4', ['internet'])
    module.includes = '.'
    module.source = [
        'model/myprotocol4-rtable.cc',
        'model/myprotocol4-packet.cc',
        'model/myprotocol4-routing-protocol.cc',
        'model/myprotocol4-id-cache.cc',
        'model/myprotocol4-rqueue.cc',
        'helper/myprotocol4-helper.cc'
        ]

    # module_test = bld.create_ns3_module_test_library('myprotocol')
    # module_test.source = [
    #     'test/myprotocol-testcase.cc',
    #     ]

    headers = bld(features='ns3header')
    headers.module = 'myprotocol4'
    headers.source = [
        'model/myprotocol4-rtable.h',
        'model/myprotocol4-packet.h',
        'model/myprotocol4-routing-protocol.h',
        'model/myprotocol4-id-cache.h',
        'model/myprotocol4-rqueue.h',
        'helper/myprotocol4-helper.h',
        ]
    if (bld.env['ENABLE_EXAMPLES']):
      bld.recurse('examples')

    bld.ns3_python_bindings()
