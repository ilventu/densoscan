#!/bin/bash

convert icon.png -resize 256x256 icon256.png
convert icon.png -resize 128x128 icon128.png
convert icon.png -resize 64x64 icon64.png
convert icon.png -resize 48x48 icon48.png
convert icon.png -resize 32x32 icon32.png
convert icon.png -resize 16x16 icon16.png

convert icon256.png icon128.png icon64.png icon32.png icon16.png icon.ico
convert icon256.png icon128.png icon64.png icon32.png icon16.png icon.icns
