SET WORKDIR=%CD%
SET LINUXHOME=%USERPROFILE:~2%
SET LINUXHOME=%LINUXHOME:\=/%
SET LINUXWORKDIR=%WORKDIR:~2%
SET LINUXWORKDIR=%LINUXWORKDIR:\=/%

REM        --mount src=//%USERPROFILE:\=/%/.aws,target=/Users/%USERNAME%/.aws,type=volume,readonly

docker run -it --rm ^
       -e HOME=%LINUXHOME% ^
       -e USER=%USERNAME% ^
       -e WORKDIR=%LINUXWORKDIR% ^
       --net=host ^
       --cap-add=NET_ADMIN ^
       --mount src=/%WORKDIR:\=/%,target=%LINUXWORKDIR%,type=bind ^
       -v %USERPROFILE%\.aws:%LINUXHOME%/.aws ^
       --workdir %LINUXWORKDIR% ^
       cloudrobotics_tutorial

