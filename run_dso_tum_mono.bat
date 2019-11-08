cd "%~dp0"
set PATH=build/examples/Release;%PATH%
set GLOG_minloglevel=0
run_dso_tum_mono.exe preset=0 quiet=1 files=sequence_33/images.zip vignette=sequence_33/vignette.png calib=sequence_33/camera.txt gamma=sequence_33/pcalib.txt noros=0 nolog=1 prefetch=1 loopclosing=1