v:
cd V:\wokwi-projects\wokwi-LN298module-chip
wokwi-cli chip compile src\chip-moduleLN298.chip.c
cd dist
copy ..\src\chip-moduleLN298.chip.wasm chip.wasm 
copy ..\src\chip-moduleLN298.chip.json chip.json

tar.exe -acf chip.zip chip.json chip.wasm
cd ..
