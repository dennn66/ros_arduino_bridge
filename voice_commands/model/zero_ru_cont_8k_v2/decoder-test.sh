#!/bin/sh

# This is a simple example of the decoder invocation
# Please note, that recognize the file should be single-channel with sample rate 8kHz
# decoder-test.wav: RIFF (little-endian) data, WAVE audio, Microsoft PCM, 16 bit, mono 8000 Hz
# Expected result for the test file is: ил -ья ильф евген -ий петр -ов золот -ой тел -ёнок

pocketsphinx_continuous \
    -samprate 8000 \
    -lm zero_ru.lm \
    -dict zero_ru.dic \
    -hmm zero_ru.cd_cont_4000 \
    -logfn /dev/null \
    -infile decoder-test.wav
