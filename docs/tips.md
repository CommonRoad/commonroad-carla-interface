
## Running pygame without physical monitor
If you are developing on a virtual machine and want to use off-screen mode, active 
`config.simulation.ignore_video_driver`, this sets the SDL to use the dummy NULL video driver so pygame can run 
without a windowing system.
