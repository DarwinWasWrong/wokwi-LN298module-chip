v:
cd V:\wokwi-projects\wokwi-LN298module-chip
wokwi-cli chip compile src\chip-moduleLN298.chip.c
copy V:\wokwi-projects\wokwi-LN298module-chip\src\chip-moduleLN298.chip.wasm V:\wokwi-projects\wokwi-LN298module-chip\dist\chip.wasm 
copy V:\wokwi-projects\wokwi-LN298module-chip\src\chip-moduleLN298.chip.json V:\wokwi-projects\wokwi-LN298module-chip\dist\chip.json
 tar.exe -a -c -f V:\wokwi-projects\wokwi-LN298module-chip\dist\chip.zip V:\wokwi-projects\wokwi-LN298module-chip\src\chip-moduleLN298.chip.json V:\wokwi-projects\wokwi-LN298module-chip\src\chip-moduleLN298.chip.wasm V:\wokwi-projects\wokwi-LN298module-chip\dist\chip.wasm 