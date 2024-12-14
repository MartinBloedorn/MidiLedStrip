#include <Adafruit_NeoPixel.h>

#include <array>

#include "Colors.h"

#define LED_PIN 0
#define LED_COUNT 280       //18
#define LED_BRIGHTNESS 255  //40  // (max = 255) ~>  max 10mA per LED

// https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


/* * * * * * NOTETOSTRIP * * * * * * * * * * * * * * * * * * * * * * * * * * */


class NoteToStrip {
public:
  static const uint8_t CC_STRIP_LENGTH = 14;      // todo
  static const uint8_t CC_STRIP_BRIGTHNESS = 15;  // todo + debounce time

  static const uint8_t CC_NOTE_MIN = 16;
  static const uint8_t CC_NOTE_MAX = 17;
  static const uint8_t CC_STRIP_MIN = 18;
  static const uint8_t CC_STRIP_MAX = 19;

  static const uint8_t CC_HUE = 20;
  static const uint8_t CC_WIDTH = 21;
  static const uint8_t CC_ADSR_A = 22;
  static const uint8_t CC_ADSR_S = 23;
  static const uint8_t CC_ADSR_R = 24;

  static const uint8_t CC_MOD_VEL_HUE = 25;
  static const uint8_t CC_MOD_VEL_WIDTH = 26;
  static const uint8_t CC_MOD_VEL_AR = 27;
  static const uint8_t CC_MOD_VEL_S = 28;

  static constexpr float ADSR_CC_TO_MS = 400.0;

private:
  struct NoteState {
    uint8_t note = 0;
    uint8_t vel = 0;
    uint8_t ch = 0;
    bool r = false;
    uint32_t tstamp = 0;  // timestamp of onset/release
  };

  struct ADSR {
    float a = 100.0;  // ms
    float s = 1.0;    // [0.0, 1.0]
    float r = 100.0;  // ms
  };

  struct ChannelSettings {
    uint8_t noteMin = 36;  // C2
    uint8_t noteMax = 60;  // C4
    float rangeMin = 0.0;
    float rangeMax = 1.0;

    float hue = 0.0;     // degrees
    float width = 0.03;  // >=0.0
    ADSR adsr;

    struct {
      float hue = 0.0;
      float width = 0.0;
      float ar = 0.0;
      float s = 0.0;
    } mod;  // all values [0.0, 1.0]
  };

  Adafruit_NeoPixel* m_strip = nullptr;

  std::array<NoteState, 48> m_notes;

  std::array<ChannelSettings, 16> m_channelSettings;

  // https://en.wikipedia.org/wiki/Gaussian_function
  float gaussian(float x, float b, float c) {
    return exp(-pow(x - b, 2.0) / (2.0 * c * c));
  }

  float normMidiVal(uint8_t val) {
    return float(constrain(val, 0, 127)) / 127.0;
  }

  void removeFinishedNotes(uint32_t tnow) {
    for (size_t i = 0; i < m_notes.size(); ++i) {
      if (m_notes[i].note  // note exists...
          && (
            // and was released enough time ago, or...
            (m_notes[i].r && tnow - m_notes[i].tstamp > m_channelSettings[m_notes[i].ch].adsr.r)
            // note is no longer within the min/max boundary
            || m_notes[i].note < m_channelSettings[m_notes[i].ch].noteMin
            || m_notes[i].note > m_channelSettings[m_notes[i].ch].noteMax)) {
        m_notes[i].note = 0;
      }
    }
  }

public:
  NoteToStrip() = default;

  void setStrip(Adafruit_NeoPixel* strip) {
    m_strip = strip;
  }

  void dumpChannelSettings(uint8_t ch) {
    ch = constrain(ch, 0, 15);
    auto& settings = m_channelSettings[ch];

    auto print = [](auto& label, auto& value) {
      Serial.print(label);
      Serial.print(" = ");
      Serial.println(value);
    };

    print("CHANNEL     ", ch);
    print("    noteMin ", settings.noteMin);
    print("    noteMax ", settings.noteMax);
    print("    rangeMin", settings.rangeMin);
    print("    rangeMax", settings.rangeMax);
    print("    hue     ", settings.hue);
    print("    width   ", settings.width);
    print("    ADSR.a  ", settings.adsr.a);
    print("    ADSR.s  ", settings.adsr.s);
    print("    ADSR.r  ", settings.adsr.r);

    print("mod.hue     ", settings.mod.hue);
    print("mod.width   ", settings.mod.width);
    print("mod.ADSR.ar ", settings.mod.ar);
    print("mod.ADSR.s  ", settings.mod.s);
  }

  void onControlChanged(uint8_t ch, uint8_t control, uint8_t value) {
    ch = constrain(ch, 0, 15);

    switch (control) {
      case CC_NOTE_MIN:
        m_channelSettings[ch].noteMin = value;
        break;
      case CC_NOTE_MAX:
        m_channelSettings[ch].noteMax = value;
        break;
      case CC_STRIP_MIN:
        m_channelSettings[ch].rangeMin = normMidiVal(value);
        break;
      case CC_STRIP_MAX:
        m_channelSettings[ch].rangeMax = normMidiVal(value);
        break;

      case CC_HUE:
        m_channelSettings[ch].hue = normMidiVal(value) * 360.0;
        break;
      case CC_WIDTH:
        m_channelSettings[ch].width = normMidiVal(value) / 10.0;
        break;
      case CC_ADSR_A:
        m_channelSettings[ch].adsr.a = normMidiVal(value) * ADSR_CC_TO_MS;
        break;
      case CC_ADSR_S:
        m_channelSettings[ch].adsr.s = normMidiVal(value);
        break;
      case CC_ADSR_R:
        m_channelSettings[ch].adsr.r = normMidiVal(value) * ADSR_CC_TO_MS;
        break;

      case CC_MOD_VEL_HUE:
        m_channelSettings[ch].mod.hue = normMidiVal(value);
        break;
      case CC_MOD_VEL_WIDTH:
        m_channelSettings[ch].mod.width = normMidiVal(value);
        break;
      case CC_MOD_VEL_AR:
        m_channelSettings[ch].mod.ar = normMidiVal(value);
        break;
      case CC_MOD_VEL_S:
        m_channelSettings[ch].mod.s = normMidiVal(value);
        break;

      default:
        break;
    };
  }

  // vel > 0: note ON; vel == 0: note OFF
  void onNoteChanged(uint8_t ch, uint8_t note, uint8_t vel) {
    ch = constrain(ch, 0, 15);

    if (note < m_channelSettings[ch].noteMin || note > m_channelSettings[ch].noteMax)
      return;

    size_t n = m_notes.size();
    size_t found_index = n;
    size_t empty_index = n;

    for (size_t i = 0; i < n; ++i) {
      if (m_notes[i].note == note && m_notes[i].ch == ch) {
        found_index = i;
        break;
      }
      if (empty_index == n && !m_notes[i].note)
        empty_index = i;
    }

    if (size_t i = found_index < n ? found_index : empty_index; i < n) {
      m_notes[i].note = note;
      m_notes[i].vel = vel;
      m_notes[i].ch = ch;
      m_notes[i].r = (vel == 0);
      m_notes[i].tstamp = millis();
    }
  }

  RGB getNoteColor(uint8_t ch, uint8_t note, uint8_t vel) {
    (void)note;
    float hue = m_channelSettings[ch].hue;
    hue += normMidiVal(vel) * m_channelSettings[ch].mod.hue * 360.0;  // modulation
    hue = hue > 360.0 ? hue - 360.0 : hue;                            // wrap around if needed
    return hsv2rgb(HSV{ hue, 1.0, 1.0 });
  }

  double getNoteWidth(uint8_t ch, uint8_t note, uint8_t vel) {
    (void)note;
    float width = m_channelSettings[ch].width;
    width *= (1.0 - (1.0 - normMidiVal(vel)) * m_channelSettings[ch].mod.width);
    return width;
  }

  // todo: create a v2 that interpolates between the previous envelope state,
  // which should be part of NoteState
  double computeEnvelope(uint32_t t, uint8_t ch, uint8_t vel, uint32_t tonset, bool r) {
    auto& settings = m_channelSettings[ch];

    float dt = t - tonset;
    float rampt = r ? settings.adsr.r : settings.adsr.a;
    rampt *= (1.0 - settings.mod.ar * normMidiVal(vel));  // AR modulation

    float k = min((dt + 1.0) / (rampt + 1.0), 1.0);
    k = r ? 1.0 - k : k;
    k *= settings.adsr.s * (1.0 - settings.mod.s * (1.0 - normMidiVal(vel)));  // S modulation

    return k;
  }

  RGB evaluate(uint32_t t, float x) {
    RGB result;

    for (size_t i = 0; i < m_notes.size(); ++i) {
      NoteState& entry = m_notes[i];

      if (entry.note) {
        ChannelSettings& settings = m_channelSettings[entry.ch];

        float scaling = settings.rangeMax - settings.rangeMin;
        if (scaling > 0.0 && (x < settings.rangeMin || x > settings.rangeMax)) continue;
        if (scaling < 0.0 && (x < settings.rangeMax || x > settings.rangeMin)) continue;

        // center point of the note on the [0, 1] strip
        float center = float(entry.note - settings.noteMin) / float(settings.noteMax - settings.noteMin);
        center = center * scaling + settings.rangeMin;

        double width = getNoteWidth(entry.ch, entry.note, entry.vel);
        RGB color = getNoteColor(entry.ch, entry.note, entry.vel);
        float env = computeEnvelope(t, entry.ch, entry.vel, entry.tstamp, entry.r);

        color.a = env * gaussian(x, center, width * scaling);
        result = rgbAdd(color, result);
      }
    }
    return result;
  }

  void update() {
    uint32_t t = millis();
    uint16_t n = strip.numPixels();

    for (int i = 0; i < n; i++) {
      float x = float(i) / float(n - 1);
      RGB rgb = evaluate(t, x);
      strip.setPixelColor(i, strip.Color(255.0 * rgb.a * rgb.r, 255.0 * rgb.a * rgb.g, 255.0 * rgb.a * rgb.b));
    }

    strip.show();
    removeFinishedNotes(t);
  }
};


/* * * * * * STATIC FUNCTIONS  * * * * * * * * * * * * * * * * * * * * * * * */


NoteToStrip noteToStrip;

// NOTE: MIDI library uses channels in [1, 16]

void onMIDIControlChanged(uint8_t channel, uint8_t control, uint8_t value) {
  Serial.print(" cc  :: ");
  Serial.print(control);
  Serial.print(", ");
  Serial.print(value);
  Serial.print(", ");
  Serial.println(channel);

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  noteToStrip.onControlChanged(channel - 1, control, value);
}

void onMIDINoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  Serial.print(" on  :: ");
  Serial.print(note);
  Serial.print(", ");
  Serial.print(velocity);
  Serial.print(", ");
  Serial.println(channel);

  digitalWrite(LED_BUILTIN, HIGH);
  noteToStrip.onNoteChanged(channel - 1, note, velocity);

  if (note == 0) {
    noteToStrip.dumpChannelSettings(channel - 1);
  }
}

void onMIDINoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
  Serial.print(" off :: ");
  Serial.print(note);
  Serial.print(", ");
  Serial.print(0);
  Serial.print(", ");
  Serial.println(channel);

  digitalWrite(LED_BUILTIN, LOW);
  noteToStrip.onNoteChanged(channel - 1, note, 0);
}


/* * * * * * MAIN  * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * TODOs:
 * - Interpolate ASR when note gets retriggered (see computeEnvelope)
 * - Store parameters in EEPROM.
 */

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  strip.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();   // Turn OFF all pixels ASAP
  strip.setBrightness(LED_BRIGHTNESS);

  noteToStrip.setStrip(&strip);

  // https://www.pjrc.com/teensy/td_midi.html
  usbMIDI.setHandleControlChange(onMIDIControlChanged);
  usbMIDI.setHandleNoteOff(onMIDINoteOff);
  usbMIDI.setHandleNoteOn(onMIDINoteOn);
}

void loop() {
  while (usbMIDI.read()) {}
  noteToStrip.update();
  delayMicroseconds(500);
}
