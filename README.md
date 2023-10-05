# Programarea_apliicatilor_de_timp_real_proiect

Să se proiecteze şi să se realizeze o aplicaţie încorporată bazată pe sistemul de operare FreeRTOS şi microcontroller-ul dsPIC33FJ128MC802. 
Aplicaţia va comanda o trapă prin intermediul unui servomotor, pe baza informaţiei de temperatură achiziţionate prin intermediul unui senzor de temperatură digital DS18S20/DS18B20.
Aplicaţia va respecta următoarele detalii de implementare:
1.va fi pornită respectiv oprită prin intermediul butonului SW1 de pe plăcuţă. Starea aplicaţiei va fi indicată de un LED conectat la pinul RB0. 
LED-ul va fi aprins atunci când aplicaţia este pornită şi va fi aprins intermitent când aplicaţia este oprită.
2.temperatura va fi achiziţionată de senzorul DS18S20 sau DS18B20, deschiderea trapei fiind controlată prin servomotor. Temperaturii de 25°C îi va corespunde poziţia centrală a servomotorului. 
Servomotorul se va deplasa între poziţiile extreme pentru limitele de temperatură (20°C-30°C). Senzorul de temperatură va fi conectat la pinul RB2 al microcontroller-ului.
3.aplicaţia va pune la dispoziţie pe lângă modul automat de comandă a servomotorului şi un mod manual de control prin intermediul unei tensiuni externe aplicate pe pinul RB3. 
Trecerea dintr-un mod de lucru în altul se face prin intermediul meniului de comenzi implementat pe PC. 
În modul de lucru manual servomotorul va fi poziţionat pe mijloc pentru o tensiune de 2V şi se va deplasa în poziţiile extreme pentru valorile limită de 1V şi 3V. 
Modul de lucru va fi indicat vizual prin intermediul unui LED conectat la pinul RB1.
LED-ul va fi aprins în modul de lucru automat şi stins în modul de lucru manual.
4.aplicaţia va permite comunicaţia prin interfaţa serială cu PC-ul. 
Placa cu dsPIC va recepţiona comenzi sub forma unor caractere şi va transmite drept răspuns şiruri de caractere.
5.pe un ecran LCD vor fi afişate informaţii privind temperatura, modul de lucru, tensiunea achiziţionată de la pinul RB3 precum şi ultima comandă primită.
6.aplicaţia va pune la dispoziţia utilizatorului un meniu cu următoarele comenzi:
interogare mod de lucru;
comutare mod de lucru automat/manual (modul de lucru predefinit este cel automat);
interogare temperatură.
