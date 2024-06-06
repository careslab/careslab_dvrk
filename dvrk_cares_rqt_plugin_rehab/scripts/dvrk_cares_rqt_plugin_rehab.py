#!/usr/bin/env python

import sys

from dvrk_cares_rqt_plugin_rehab.careslab_module import MyPlugin
from rqt_gui.main import Main

plugin = 'dvrk_cares_rqt_plugin_rehab'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
