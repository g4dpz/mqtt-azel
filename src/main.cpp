#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <CSV_Parser.h>
#include <math.h>
#include "gs-232b.hpp"

#define RXD2 16
#define TXD2 17

const char* ssid = "badgersoft-iot";
const char* password = "h4les0wen1234";

const char* mqtt_server = "data.b2-space.com";
const char* mqtt_user = "mosquitto";
const char* mqtt_pass = "B2Space!";

// set these to the observer location
float refLat = 52.4674;
float refLong = -2.02150;
float refAlt = 208.0;

typedef struct vector8 {
  float x;
  float y;
  float z;
  float radius;
  float nx;
  float ny;
  float nz;
  bool err;
} vector8;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[255];

GS232B gs232b;

float calcEarthRadiusInMeters(float latitudeRadians)
{
    // latitudeRadians is geodetic, i.e. that reported by GPS.
    // http://en.wikipedia.org/wiki/Earth_radius
    float a = 6378137.0;  // equatorial radius in meters
    float b = 6356752.3;  // polar radius in meters
    float c = cos(latitudeRadians);
    float s = sin(latitudeRadians);
    float t1 = a * a * c;
    float t2 = b * b * s;
    float t3 = a * c;
    float t4 = b * s;
    return sqrt((t1*t1 + t2*t2) / (t3*t3 + t4*t4));
}

float calcGeocentricLatitude(float lat)
{
    // Convert geodetic latitude 'lat' to a geocentric latitude 'clat'.
    // Geodetic latitude is the latitude as given by GPS.
    // Geocentric latitude is the angle measured from center of Earth between a point and the equator.
    // https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude
    float e2 = 0.00669437999014;
    float clat = atan((1.0 - e2) * tan(lat));
    return clat;
}


float calcDistance (vector8 ap, vector8 bp)
{
    float dx = ap.x - bp.x;
    float dy = ap.y - bp.y;
    float dz = ap.z - bp.z;
    return sqrt (dx*dx + dy*dy + dz*dz);
}

struct vector8 convertLocationToPoint(struct vector8 l)
{
    // Convert (lat, lon, elv) to (x, y, z).
    float lat = l.x * PI / 180.0;
    float lon = l.y * PI / 180.0;
    float radius = calcEarthRadiusInMeters(lat);
    float clat   = calcGeocentricLatitude(lat);

    float cosLon = cos(lon);
    float sinLon = sin(lon);
    float cosLat = cos(clat);
    float sinLat = sin(clat);

    struct vector8 p;
    p.x = radius * cosLon * cosLat;
    p.y = radius * sinLon * cosLat;
    p.z = radius * sinLat;

    // We used geocentric latitude to calculate (x,y,z) on the Earth's ellipsoid.
    // Now we use geodetic latitude to calculate normal vector from the surface, to correct for elevation.
    float cosGlat = cos(lat);
    float sinGlat = sin(lat);

    float nx = cosGlat * cosLon;
    float ny = cosGlat * sinLon;
    float nz = sinGlat;

    p.nx = nx;
    p.ny = ny;
    p.nz = nz;

    p.x += l.z * nx;
    p.y += l.z * ny;
    p.z += l.z * nz;

    // check for possible Z NaN
    if (isnan(p.z)) {p.z = 0.0;}

    p.radius = radius;

    return p;

}

struct vector8 rotateGlobe (vector8 b, vector8 a, float bradius, float aradius)
{
    // Get modified coordinates of 'b' by rotating the globe so that 'a' is at lat=0, lon=0.
    vector8 br;
    br.x = b.x;
    br.y = b.y - a.y;
    br.z = b.z;
    vector8 brp = convertLocationToPoint(br);

    // Rotate brp cartesian coordinates around the z-axis by a.lon degrees,
    // then around the y-axis by a.lat degrees.
    // Though we are decreasing by a.lat degrees, as seen above the y-axis,
    // this is a positive (counterclockwise) rotation (if B's longitude is east of A's).
    // However, from this point of view the x-axis is pointing left.
    // So we will look the other way making the x-axis pointing right, the z-axis
    // pointing up, and the rotation treated as negative.

    float alat = calcGeocentricLatitude(-a.x * PI / 180.0);
    float acos = cos(alat);
    float asin = sin(alat);

    vector8 rg;

    rg.x = (brp.x * acos) - (brp.z * asin);
    rg.y = brp.y;
    rg.z = (brp.x * asin) + (brp.z * acos);
    rg.radius = bradius;

    return rg;
}

struct vector8 normalizeVectorDiff(vector8 b, vector8 a)
{
    vector8 nvd;

    nvd.err = false;

    // Calculate norm(b-a), where norm divides a vector by its length to produce a unit vector.
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dz = b.z - a.z;
    float dist2 = dx*dx + dy*dy + dz*dz;
    if (abs(dist2) < 0.00001) {
        nvd.err = true;
        return nvd;
    }
    float dist = sqrt(dist2);
    nvd.x = (dx/dist);
    nvd.y = (dy/dist);
    nvd.z = (dz/dist);
    nvd.radius = 1.0;
    return nvd;
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("TrackingClient", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("lora_data");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  char messageTemp[255];
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp[i] += (char)message[i];
    messageTemp[i+1] = '\0';
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "lora_data") {

    // 13:49:57 Ch1: $$KSF_TST,16011,13:49:56,52.46738,-2.02142,199,301,359.0*09E8

    vector8 obsLoc;
    obsLoc.x = refLat;
    obsLoc.y = refLong;
    obsLoc.z = refAlt;
    vector8 obsPoint = convertLocationToPoint(obsLoc);

    CSV_Parser cp(messageTemp, /*format*/ "sLsfffLs", false, ',', '"');
    float* latitude = (float*)cp[3];
    float* longitude = (float*)cp[4];
    float* altitude = (float*)cp[5];

    vector8 payLoc;
    payLoc.x = latitude[0];
    payLoc.y = longitude[0];
    payLoc.z = altitude[0];
    vector8 payPoint = convertLocationToPoint(payLoc);

    float distance = calcDistance(obsPoint, payPoint) * 0.001;

    Serial.print("Distance: ");
    Serial.println(distance);

    // Let's use a trick to calculate azimuth:
    // Rotate the globe so that point A looks like latitude 0, longitude 0.
    // We keep the actual radii calculated based on the oblate geoid,
    // but use angles based on subtraction.
    // Point A will be at x=radius, y=0, z=0.
    // Vector difference B-A will have dz = N/S component, dy = E/W component.
    float azimuth = 0.0;
    vector8 br = rotateGlobe (payLoc, obsLoc, payPoint.radius, obsPoint.radius);
    if (br.z*br.z + br.y*br.y > 1.0e-6) {
        float theta = atan2(br.z, br.y) * 180.0 / PI;
        azimuth = 90.0 - theta;
        if (azimuth < 0.0) {
            azimuth += 360.0;
        }
        if (azimuth > 360.0) {
            azimuth -= 360.0;
        }
        Serial.print("Azimuth: ");
        Serial.println(azimuth);
    }

    float elevation = 0.0;
    vector8 bma = normalizeVectorDiff(payPoint, obsPoint);
    if (!bma.err) {
        // Calculate elevation, which is the angle above the horizon of B as seen from A.
        // Almost always, B will actually be below the horizon, so the elevation will be negative.
        // The dot product of bma and norm = cos(zenith_angle), and zenith_angle = (90 deg) - elevation.
        // So elevation = 90 - acos(dotprod).
        float elevation = 90.0 - (180.0 / PI)*acos(bma.x*obsPoint.nx + bma.y*obsPoint.ny + bma.z*obsPoint.nz);
        
        Serial.print("Elevation: ");
        Serial.println(elevation);
    }

    gs232b.setPosition(azimuth, elevation);

  }
}


void setup() {
  
  Serial.begin(115200);
  
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  gs232b.init(&Serial2);

  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}