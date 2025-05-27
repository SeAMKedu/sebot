### Arduino

Arduinon koodi löytyy hakemistosta ``/opt/nomga/arduino/``

```bash
# Käännä koodi
compile.sh

# Lataa arduinoon
upload.sh
```

### ROS2

ROS2 koodit löytyvät hakemistosta ``/opt/nomga/ros2/``

Toimintaa voi testata

```bash
source /opt/nomga/ros2_ws/install/setup.bash
```

Hakemistosta ``/opt/nomga/ros2_ws/src/diffdrive/diffdrive/`` löytyy vielä ``pi_led.py`` tiedosto, joka sytyttää Raspberry PI GPIO23:een kytketyn ledin kun ajetaan ``ros2 launch diffdrive diffrive.launch.py`` ja kaikki on valmista. Huomaa, että kyseessä ei ole varsinainen ROS2 paketti eli sitä ei ajeta ros2 run ... komennolla.


Näiden .md tiedostojen jakaminen suljetussa lähiverkossa onnistuu code-kansiosta löytyvällä python-skriptillä serve_md_files.py, jonka hakemiston alihakemistossa /markdown_files on kopioituna nämä md-tiedostot.

-
Nomga Oy - SeAMK - ROS 2 ja moottorinohjaus: PWM-signaalista robottien liikkeenhallintaan