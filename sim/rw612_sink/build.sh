#!/usr/bin/env bash
# Build simulator Sink Node dari kode Java ASLI repo npa-vee-rw612.
#
#   sim/rw612_sink/build.sh /path/ke/npa-vee-rw612
#
# Yang dipakai apa adanya dari firmware:
#   services/mqtt/MqttSubscribeService.java, services/mqtt/JsonLite.java,
#   models/actuator/*, models/sensor/SensorData.java
# Satu-satunya perubahan: BROKER_URI bisa dioverride dengan -Dsim.broker=...
# (default tetap tcp://168.110.214.70:1883 seperti di board).
# API MicroEJ (ej.bon.Util, ej.microui.MicroUI, micropaho MqttCallback) dan
# controller UI diganti stub di folder stub/.
set -euo pipefail
RW612_REPO="${1:?pakai: build.sh /path/ke/npa-vee-rw612}"
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$HERE/build"
SRC_ROOT="$RW612_REPO/apps/SmartGreenhouse/src/main/java"
PKG=com/nxp/example/smartgreenhouse
PAHO_VER=1.2.5
PAHO_URL="https://repo1.maven.org/maven2/org/eclipse/paho/org.eclipse.paho.client.mqttv3/$PAHO_VER/org.eclipse.paho.client.mqttv3-$PAHO_VER.jar"

rm -rf "$BUILD/src" "$BUILD/classes"
mkdir -p "$BUILD/src/$PKG/services/mqtt" "$BUILD/src/$PKG/models" "$BUILD/classes" "$BUILD/lib"

cp "$SRC_ROOT/$PKG/services/mqtt/JsonLite.java" "$BUILD/src/$PKG/services/mqtt/"
sed 's#private static final String BROKER_URI = "tcp://168.110.214.70:1883";#private static final String BROKER_URI = System.getProperty("sim.broker", "tcp://168.110.214.70:1883");#' \
    "$SRC_ROOT/$PKG/services/mqtt/MqttSubscribeService.java" > "$BUILD/src/$PKG/services/mqtt/MqttSubscribeService.java"
if ! grep -q 'System.getProperty("sim.broker"' "$BUILD/src/$PKG/services/mqtt/MqttSubscribeService.java"; then
    echo "WARNING: BROKER_URI di firmware berubah; simulator akan konek ke broker asli di firmware." >&2
fi
cp -r "$SRC_ROOT/$PKG/models/actuator" "$SRC_ROOT/$PKG/models/sensor" "$BUILD/src/$PKG/models/"

if [ ! -f "$BUILD/lib/paho-unsigned.jar" ]; then
    curl -sSfL -o "$BUILD/lib/paho.jar" "$PAHO_URL"
    cp "$BUILD/lib/paho.jar" "$BUILD/lib/paho-unsigned.jar"
    # Jar resmi di-sign; signature dibuang agar interface MqttCallback bisa diganti
    # versi micropaho (2 method) dari stub/.
    zip -q -d "$BUILD/lib/paho-unsigned.jar" 'META-INF/*.SF' 'META-INF/*.RSA'
fi

javac -nowarn -d "$BUILD/classes" -cp "$BUILD/lib/paho-unsigned.jar" \
      -sourcepath "$BUILD/src:$HERE/stub" \
      "$HERE/stub/SinkNodeSim.java" "$HERE/stub/org/eclipse/paho/client/mqttv3/MqttCallback.java"
echo "OK: jalankan dengan"
echo "  java -Dsim.broker=tcp://127.0.0.1:1883 -cp $BUILD/classes:$BUILD/lib/paho-unsigned.jar SinkNodeSim"
