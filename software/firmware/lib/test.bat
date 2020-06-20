@echo off
set PATH=C:\MinGW\bin;%PATH%
@echo on

del a.exe
g++ %1 -I../gtest -L../gtest -lgtest -D_UNIT_TEST

a.exe