Stacja pogodowa APRS przy wykorzystaniu urządzenia  Hydrawise Garni 2055 ARCUS

![ba902190f1ad4248ac2cc03eb01fbc54](https://github.com/user-attachments/assets/81e40a49-57ca-4203-8acc-2eecd520e0b7)


 Główny autor oprogranowania: 
 
 SP3WRO https://github.com/SP3WRO/BresserWeatherSensorBasicAPRS
 
 
 Integracja: Bresser 7-in-1 + ESP8266 + BME280 + CC1101 -> APRS (TCP) + WebUI
 Biblioteka: BresserWeatherSensorReceiver v0.37.0 (podstawa)
 
 Interfejs WWW by SP3VSS 2k26
 
 Wpisz w "KONFIGURACJA APRS I WIFI" swoje dane, potem możesz je zmień poprzez stronę WWW

Program w wesji z stroną www:

![rodosWX](https://github.com/user-attachments/assets/97ae1456-0034-4ebc-8b1f-3960a0087f15)


Opis autora:

RodosWX_2 - jedyny słuszny program dla stacji pogodowych garni/bresser wysyłający ramki do aprs-is. Testowany na ludziach po spożyciu i przed spożyciem dużych dawek alkoholu.

Uwaga, program napisany WYŁĄCZNIE dla ESP8266 + CC1101 + BME280 (dla ciśnienia, którego bresser nie podaje) Co i jak:

Instalujemy Arduino IDE
W Arduino IDE instalujemy biblioteki: BresserWeatherSensorReceiver (0.37.0) + wszystkie pakiety wymagane oraz Adafruit BME280
Oprócz tego instalujemy płytkę ESP8266.
Przechodzimy do C:/Users/twoja_nazwa/Documents/Arduino/libraries/BresserWeatherSensorReceiver/src
Nadpisujemy plik WeatherSensorCfg.h
Włączamy Arduino IDE i wybieramy:
File > Examples > BresserWeatherSensorReceiver > BresserWeatherSensorBasic
Usuwamy całą zawartość i wklejamy z naszego pliku BresserWeatherSensorBasicAPRS.ino
Zmieniamy dane APRS i lokalizację.
Kompilujemy program i wrzucamy do urządzenia. Diagnostyka przez monitor szeregowy na 115200. (Informacje o inicjalizacji urządzenia, odebranych pakietach, podłączeniu BME, Wi-Fi, pobraniu czasu z NTP)
Pierwsza ramka aprs idzie dopiero po 15 minutach od uruchomienia!


Połączenia:

BME280:
SDA  -> GPIO2

SCL  -> GPIO5

3.3V -> 3.3V

GND  -> GND


CC1101:
GDO0 -> GPIO4

SCK  -> GPIO14

MISO -> GPIO12

MOSI -> GPIO13

CS   -> GPIO15

3.3V -> 3.3V

GND  -> GND


Program działa w pętli nieskończonej, realizując dwa główne zadania: ciągły nasłuch radia (zbieranie danych) oraz okresowe wysyłanie raportu (co 15 minut).

Oto szczegółowy opis logiki działania:

Ciągły nasłuch (Pętla główna loop)
ESP8266 przez 99% czasu nasłuchuje sygnałów radiowych na częstotliwości 868 MHz. Dzieje się to w funkcji loop():

Odbiór pakietu: Gdy stacja pogodowa (Bresser 7-in-1) wyśle sygnał (dzieje się to co kilkanaście sekund), moduł CC1101 go wyłapuje.

Wstępne przetwarzanie:

Temperatura, Wilgotność, UV, Światło są zapisywane do zmiennej tymczasowej current_wx (zawsze nadpisujemy je najświeższą wartością).

Matematyka wiatru (Kluczowy moment):

Stacja wysyła prędkość wiatru w danej sekundzie. Program nie może wysłać tylko tej jednej wartości, bo wiatr jest zmienny.

Średnia: Każdy odebrany pomiar prędkości jest dodawany do sumy (wind_speed_sum), a licznik próbek (wind_sample_count) wzrasta o 1.

Porywy (Gust): Program sprawdza, czy aktualny poryw wiatru jest większy od zapamiętanego maksimum. Jeśli tak – aktualizuje rekordzistę (wind_gust_max_period).

Wyzwalacz czasowy (Co 15 minut)
Program sprawdza zegar systemowy. Gdy minie 15 minut (REPORT_INTERVAL_MS) od ostatniego raportu, uruchamiana jest procedura wysyłania send_aprs_frame().

Oto co dzieje się w ułamku sekundy po upływie 15 minut: A. Obliczenie Wiatru

Średnia prędkość: Program dzieli Suma prędkości / Ilość próbek. Dzięki temu do APRS trafia rzetelna średnia z całych 15 minut, a nie przypadkowy podmuch z ostatniej sekundy.

Poryw: Pobierana jest najwyższa wartość porywu zanotowana w ciągu tych 15 minut.

B. Obliczenie Deszczu (Logika "Delta 1h")

To jest najsprytniejsza część programu. Stacja Bresser wysyła całkowitą sumę opadów od momentu włożenia baterii (tzw. Total Rain). APRS wymaga jednak podania opadu z ostatniej godziny.

Używamy "bufora kołowego" (rain_buffer), który pamięta 4 wartości (bo 4 * 15 min = 1 godzina).

Program bierze: Aktualny licznik deszczu MINUS Licznik deszczu zapamiętany 4 cykle temu.

Wynik to dokładna ilość wody, która spadła w ciągu ostatniej godziny.

C. Odczyt Ciśnienia (BME280)

W tym momencie program wybudza czujnik BME280, wykonuje jeden precyzyjny pomiar ciśnienia atmosferycznego i usypia czujnik. D. Konwersja nasłonecznienia

Stacja podaje światło w luksach (Lux). APRS wymaga formatu W/m² (Wat na metr kwadratowy).

Program przelicza to w przybliżeniu: Lux * 0.0079.
Budowa i wysyłka ramki APRS
Program skleja wszystkie dane w jeden ciąg znaków (String). Wygląda to mniej więcej tak:

SP0ABC-13>APRS,TCPIP*:@261430z5259.26N/01654.74E_270/004g008t033r000h95b10132L000...

@261430z: Dzień (26) i godzina (14:30) czasu UTC (z Internetu).

5259.26N/01654.74E_: Twoja pozycja i symbol pogody (_).

270/004g008: Wiatr z zachodu (270 stopni), średnia 4 mph, porywy 8 mph.

t033: Temperatura w Fahrenheitach.

r000: Deszcz w setnych cala z ostatniej godziny.

b10132: Ciśnienie 1013.2 hPa.

Bat:OK...: Komentarz, stan baterii, UV i RodosWX_2.
Reset cyklu
Po skutecznym wysłaniu danych do serwera euro.aprs2.net:

Liczniki wiatru są zerowane (zaczynamy liczyć średnią i porywy dla kolejnych 15 minut od nowa).

Zapisywany jest czas ostatniej wysyłki.

Program wraca do punktu 1 (nasłuchu).
Podstawowy kod, który został przerobiony: https://github.com/matthias-bs/BresserWeatherSensorReceiver
