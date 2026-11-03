set_device GW5A-25B
set_option -top_module video_top
set_option -verilog_std sysv2017
set_option -synthesis_tool gowinsynthesis

add_file -type verilog {D:/zcode_work/ads1115_drv.v}
add_file -type verilog {D:/zcode_work/audio_fx.v}
add_file -type verilog {D:/zcode_work/bend_ctrl.v}
add_file -type verilog {D:/zcode_work/dds_voice.v}
add_file -type verilog {D:/zcode_work/demo_seq.v}
add_file -type verilog {D:/zcode_work/i2s_tx.v}
add_file -type verilog {D:/zcode_work/keypad_scan.v}
add_file -type verilog {D:/zcode_work/note_alloc.v}
add_file -type verilog {D:/zcode_work/porta_ctrl.v}
add_file -type verilog {D:/zcode_work/st7789_drv.v}
add_file -type verilog {D:/zcode_work/synth_core.v}
add_file -type verilog {D:/zcode_work/ttp229_drv.v}
add_file -type verilog {D:/zcode_work/viz_scope.v}
add_file -type verilog {D:/zcode_work/viz_spec.v}
add_file -type verilog {D:/zcode_work/video_top.v}
add_file -type cst {D:/zcode_work/fpga_project.cst}

run all
