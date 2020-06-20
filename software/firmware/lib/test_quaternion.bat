@echo off
set PATH=C:\MinGW\bin;%PATH%
@echo on

del a.exe
g++ Quaternion.cpp ut_Quaternion.cpp -I../gtest -L../gtest -lgtest -D_UNIT_TEST

a.exe