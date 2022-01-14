#!/usr/bin/env bash

echo "Calculate siti from yuv videos"

si_ti_py() {
    name=$1
    echo "$name"
    siti --width 352 --height 288 "$name"_cif.yuv >"$name".json
}

si_ti_cpp() {
    name=$1
    # Requires the compiled siti from https://github.com/Telecommunication-Telemedia-Assessment/SITI
    echo "$name"
    ./siti -i "$name"_cif.yuv -w 352 -h 288 -s
}

si_ti_py akiyo
si_ti_py city
si_ti_py crew
si_ti_py container
si_ti_py waterfall
si_ti_py foreman
si_ti_py football
si_ti_py hall
si_ti_py ice
si_ti_py mobile
si_ti_py mother-daughter
si_ti_py paris
si_ti_py highway
si_ti_py tempete
