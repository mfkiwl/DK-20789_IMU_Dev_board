"C:\Program Files (x86)\Atmel\Studio\7.0\AtmelStudio.exe" EMD-Core\EMD-Core-ICM207xx.cproj /rebuild release /out scripts\ICM207xx-EMDCore.txt
"C:\Program Files (x86)\Atmel\Studio\7.0\AtmelStudio.exe" EMD-App\EMD-App-ICM207xx.cproj /rebuild release /out scripts\ICM207xx-EMDApp.txt
mkdir release\ICM207xx
copy EMD-App\Release\EMD-App-ICM207xx.bin release\ICM207xx
copy EMD-App\Release\EMD-App-ICM207xx.elf release\ICM207xx