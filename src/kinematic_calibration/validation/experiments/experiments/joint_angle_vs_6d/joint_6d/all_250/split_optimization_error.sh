#! /bin/sh

cat optimization_error.csv | head -n1 > optimization_error_larm.csv &&
grep larm optimization_error.csv >> optimization_error_larm.csv &&

cat optimization_error.csv | head -n1 > optimization_error_rarm.csv &&
grep rarm optimization_error.csv >> optimization_error_rarm.csv &&

cat optimization_error.csv | head -n1 > optimization_error_lleg.csv &&
grep lleg optimization_error.csv >> optimization_error_lleg.csv &&

cat optimization_error.csv | head -n1 > optimization_error_rleg.csv &&
grep rleg optimization_error.csv >> optimization_error_rleg.csv
