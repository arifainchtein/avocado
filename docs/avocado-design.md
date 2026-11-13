# Avocado: making a Teleonome from an Egg, a Sertoli file and a sperm

*Design, started 2026-10-31. Development case: Annabelle (Chilhuacle's, `annabelle-teleonome`). Next: Chinampa, Tucci. Later: digitalgeppetto.com.*

## 1. Why now

Annabelle's Denome was made by hand, and so was every change to it since: the rename from ChinampaMonitor
(2026-10-30, many files by hand), and the DigitalStables Publisher (a one-off Python script,
`DevicesDigitalStables/denome/add_digitalstables_publisher.py`). That works for one Annabelle. It does not
work for the second Annabelle sold, nor for Chinampa and Tucci, which are also an ESP32 plus a Pi and will
need the same DigitalStables setup and more (a description of each of Chinampa's sensors, for example).

The Teleonome already has the answer, built in 2016-2020 and paused for lack of time: the **Egg**, the
**sperm**, **Sertoli** and **Fertilizer**, with the parts in **Avocado** (from Nahuatl *ahuacatl*,
testicle). This design takes it up again, with Annabelle as the first real case, and adds the step that
was always planned last: a **web page in the factory** where a Teleonome is put together by dragging parts
into a list.

**Scope (Ari, 2026-10-31).** Avocado began when the devices were Arduino Unos without WiFi: a Pi received
commands over WiFi and passed them to the Arduino driving the relay, and the Pi carried the user interface.
Now the ESP32 has WiFi and LoRa itself, so most devices need no Pi: MonoNanny, for example, becomes a product
that is a Valentino plus a Gloria. **Avocado is for the devices that have a Pi: those that analyse data in a
way that affects operations.** Annabelle is the development case; **Chinampa is the first product**, the
closest to production; Tucci follows. The eleven recipes from the Arduino years are kept in `Avocado/old/`.

**Chinampa (Ari, 2026-10-31): its own Pi, connected by RS485, not by the serial port like Annabelle, and the Pi
does analysis.** A Chinampa is a **Valentino plus the Chinampa shield**; the shield carries the RS485 chip and
a connector for the cable to the Pi, and the Pi receives through **its own shield, the KiCad project
Meninges**. So Chinampa is its own recipe, not a
device type added to Annabelle's. What it shares with Annabelle is what makes Avocado worth it: the
DigitalStables Publisher, the settings file the factory writes at Send To DS, the memory watch, the network
parts. What is its own: a microcontroller part for the ESP32 over RS485 (nothing in the framework or the
firmware does this yet: the framework knows Chinampa only as readings relayed by Annabelle,
`ChinampaDataDeserializer`), and its analysis parts. The RS485 link between the Pi and the ESP32 (protocol,
Meninges on the Pi, firmware side) is its own piece of work, in its own session; Avocado then offers it as a
part.

**To generalize from Annabelle (found 2026-10-31):** the factory names a Teleonome's devicesheart login
`<customer>-annabelle` (`TeleonomeLogin.loginFor`), and `set-devicesheart-settings` on the Pi accepts only
that form. A Chinampa's Pi needs `<customer>-chinampa` (and its product's serial number in the settings file
is Chinampa's). Both are to become `<customer>-<product type>`.

## 2. Words

| Word | What it is | Where |
|---|---|---|
| **Denome** | the DNA of a Teleonome: one JSON file (`Teleonome.denome`) the Hypothalamus runs | the Pi |
| **Egg** | a Denome named `Egg` with only what every Teleonome needs to run the Hypothalamus | `Avocado/denomes/Egg.denome` |
| **Part** (homeobox definition) | one piece of function: a microcontroller (`.mcu`), sensor (`.hsd`), actuator (`.had`), human interface (`.hud`), publisher, telepathon... Has a `Homeobox Definition Type` that names its generator | `Avocado/mcu`, `hsd`, `had`, `hud`, ... |
| **Sertoli file** | the recipe for one kind of Teleonome: the list of parts, with their settings | `Avocado/sertoli/<Name>.sertoli` |
| **Sertoli** | turns a Sertoli file into a sperm: each part's generator writes its homeobox | `Teleonome/sertoli` |
| **Sperm** | the denes to inject: homeoboxes (denes to add) and actions (create chains, carry or remove DeneWords) | `<Name>.sperm` |
| **Fertilizer** | injects a sperm into the Egg (or into a living Denome, Purpose Type `Mutate`); can undo | `Teleonome/fertilizer` |
| **Birth** | the fertilized Egg gets its name and goes onto a Pi | (to build, §6) |

**Three kinds of content, kept apart** (this is the rule the rest follows):

1. **The kind** - what every Annabelle has: parts and their settings. In the Sertoli file and the sperm.
2. **This unit** - what differs between two Annabelles: her serial number, her devicesheart login and
   password. Never in the Denome: the factory writes it to the Pi at Send To DS
   (`/home/pi/Teleonome/devicesheart.properties`, decided 2026-10-31, DevicesDigitalStables design §16.4).
3. **What she grows** - what appears while she lives: telepathons for the devices she hears (TopTank,
   Langley_W, Chinampa), Mnemosyne data, pathology. Never in the Sertoli file.

## 3. What exists and what was found (2026-10-31)

Run in a scratchpad copy, from the current sources compiled with javac against the current framework
(nothing built, nothing on a Pi):

- **Sertoli works.** `Langley.sertoli` (ArduinoMother.mcu, LangleySensors.hsd, SFTPPublisher.had) gave a
  sperm with Main Arduino, the Langley sensor, the SFTP publisher.
- **Fertilizer works.** It injected that sperm into the 2023 Egg: the parts appear where they belong
  (Components, Sensors, Actuators, Sensor Data, a Home Page panel).
- **Found on the way:**
  - The jars in `~/Data/Teleonome` (2020) are older than the sources: `Sertoli.jar` looks for `.mcu`
    files in `hsd/`. Use the sources.
  - `sertoli/src/main/java/FlowMeterEventGenerator.java` (outside the package) no longer compiles: it
    uses `TeleonomeConstants.DENECHAIN_TELEPATHONS`, now commented out in the framework.
  - `Avocado/sperm/Template.sperm` still carries an example homebox aimed at Ra and Cleo with an empty
    pointer (`@Sperm:Hypothalamus::`); Fertilizer stops on it. `Empty.sperm` works.
  - **The new being keeps the name `Egg`.** Sertoli writes `Teleonome Name` into the sperm's Purpose,
    but Fertilizer never reads it. Naming (birth) is missing.
  - Fertilizer reads the Egg from `avocado/Teleonome.denome` and writes the result to `Teleonome.denome`
    in the folder it runs in; both programs take every path from the folder they are started in, and
    end with `System.exit`. Fine on a Pi from the command line; not usable from a web application.
  - Of the three Eggs (`Egg/Teleonome.denome` 2018, `NewEgg/` 2018, `Avocado/denomes/Egg.denome` 2023)
    only the 2023 one has the Telepathons nucleus. It is the Egg; the others go to an `old` folder.

### 3.1 Every recipe and every part, run on 2026-10-31

Each `.sertoli` file in `Avocado/sertoli` run through Sertoli (from `Empty.sperm`) and Fertilizer, each in
its own fresh copy of Avocado and the 2023 Egg. **One of eleven goes all the way: Langley.**

| Recipe | Where it stops | Why |
|---|---|---|
| Langley | - | sperm and Denome made |
| WeatherInfoPanel | Fertilizer refuses the Denome (and puts the Egg back) | the Network Inspector part was not generated (below), so two pointers to it are unresolved |
| InfoBox | Sertoli | reads `.hud` files from `hsd/` (the same kind of wrong-folder bug as the old jar's `.mcu`) |
| Campaspe | Sertoli | `Valentino.mcu` has no `Homeobox Definition Type`; its sensors are written as `{"Name","Type"}` objects, which Sertoli does not read |
| GloriaField | Sertoli | uses `MicroControllers` where Sertoli now wants `Components` |
| Cleo2 | Sertoli | has no `Components` list; Sertoli requires every list, even empty |
| Cleotilde, Gertrudiz, GloriaHome, MonoNanny, Ra2 | Sertoli | the file is not valid JSON (half-written drafts: a missing comma, an empty value) |

**Fertilizer is in better shape than Sertoli:** it checks the Denome it makes (unresolved dene and
DeneWord pointers), refuses a bad one, keeps it as `Teleonome.bad.denome` and restores the Egg. That is
the check of §4.1, already there.

**The parts (`Avocado/*/*`), 51 files:**
- 7 are not valid JSON: `NetworkInspectorMicroController.mcu` (a trailing comma), `Aquabubbler1FlowSensorNetwork.hsd`, `GloriaFieldFlowSensorNetworkValues.hsd`, `Telepathon/GloriaField.tph`, `Telepathon/GloriaFieldSensors.hsd`, plus the drafts above.
- 8 name no `Homeobox Definition Type`: `Gloria.mcu`, `GloriaField.mcu`, `GloriaHome.mcu`, `Valentino.mcu`, `AmbientTempHumidity.hsd`, both `.sec`.
- 6 name a type with no generator: `AddDataTransformation` (the generator exists as `AddDataTransformationHomeboxGenerator`, spelled differently), `AquaBubblerTelepathon`, `AquaBubblerXBeeTelepathon`, and the four `.hud` files' `UserInterfaceDefinition` (Sertoli has `HumanInterfaceHomeoBoxGenerator`; to check how `.hud` files reach it).
- Generators and parts disagree on shape: `MicroControllerHomeoBoxGenerator` reads `Hardware:Storage Manager` as a string, `NetworkInspectorMicroController.mcu` has it as an object.
- `org.json` (what Sertoli parses with) accepts trailing commas and similar, so some files that Python rejects still load in Sertoli; the page (§5) writes strict JSON.

**After `SertoliEngine` (2026-10-31, to build):** the same eleven recipes again. Three go all the way
(Langley, InfoBox, Gertrudiz); every other one stops with a list of its problems, part by part, and no sperm:

| Recipe | Problems reported |
|---|---|
| Campaspe | `Valentino.mcu` has no type; `PowerStatusStructValues.hsd` lacks `Container Page Identity Pointer`; `AquaBubbler2FountainXBee.hud` not in `hud/`; no generator for `AquaBubblerTelepathon` |
| Cleo2 | six sensors lack `Container Page Identity Pointer` (the sensor generator requires it; a sensor not shown on a page should not need one) |
| Cleotilde, WeatherInfoPanel | `NetworkInspectorMicroController.mcu`: `Storage Manager` is an object, the generator wants a string |
| GloriaField, GloriaHome | `GloriaField.mcu` / `GloriaHome.mcu` have no type; GloriaHome's containers lack `Panel In Page Position` |
| MonoNanny | `FlowMeter.hsd`, `TemperaturSensor.hsd`, `Pump.had` never written |
| Ra2 | the recipe is not valid JSON |

**Part formats (2026-10-31, to build):** `PartFormats` (sertoli) holds the format of every kind of part in one
place: `PartFormats check <avocado>` lists each part's problems, `PartFormats markdown` writes
`docs/avocado-part-formats.md` (generated: edit the code, not the file). *Errors* make Sertoli fail on the part;
*palette warnings* are what the page needs to offer it (`Display Name`, `Description`, `Category`, optional
`Requires`), plus in `Configurable` the flags `Per Unit` and `Secret`. First run: **42 parts, 19 usable in a
recipe, 0 offered in the palette** (none has the palette fields yet), 23 broken: Gloria, GloriaField,
GloriaHome, Valentino `.mcu` and `AmbientTempHumidity.hsd` have no type; most sensors lack `Reporting Value
Dene Name` or have text (`"WPS"`, `""`) where the generator reads a number; the AquaBubbler and FlowSensor types
have no generator. Fixed with it: `SensorHomeoBoxGenerator` treats the page pointer and panel as optional (it
already meant to); `NetworkInspectorMicroController.mcu` (trailing comma, empty Storage/Time Manager objects);
`AddDataTransformationHomeboxGenerator` renamed `...HomeoBoxGenerator`, the spelling Sertoli looks for.
**For the Pi parts:** `MicroControllerHomeoBoxGenerator` reads every parameter's `Value` as a number; a Pi part
needs text values (the DigitalStables Server address), so it has to follow each parameter's `Value Type`.

**Parts sorted (2026-10-31, Ari's OK):** the Arduino-era parts are in `Avocado/old/` (mcu, hsd, had, hud,
telepathons, the old `Telepathon/` folder). Left: 13 parts, **all 13 usable in a recipe**, after:
`MicroControllerHomeoBoxGenerator` writes each parameter's value as written, with its own `Value Type` (it read
every value as a number and wrote it as a String; the ranges it read were never used); a setting's path can name
a parameter, `Software:Parameters:<name>` = that parameter's `Value` (checker and recipe settings alike); the
`Configurable` paths of `CSVPublisherMicroController.mcu` and `NetworkInspectorMicroController.mcu` corrected
(the latter's pointed at no parameter; its default 8 and range 1-20 are `ArpScanRetryCount`'s). **Now (2026-11-01): 18 parts, all usable, 5 offered in the palette** (Annabelle's microcontroller, sensor and actuator, the DigitalStables Publisher, Telepathon support).

**What follows for phase 1:**
1. One schema per kind of part (`.mcu`, `.hsd`, `.had`, `.hud`, telepathon, publisher), written down,
   and a check that every part file follows it (the same check the page uses before showing a part in
   the palette). Parts that fail are not offered.
2. Sertoli: every list in a recipe optional; one folder per kind, looked up by the file's extension;
   a missing generator or a failed part is an error that stops the run with the part's name, not a
   warning followed by a half-made sperm.
3. **Done 2026-10-31:** all eleven recipes moved to `Avocado/old/sertoli/` (none is a Teleonome to build
   again, §1 Scope). The parts follow once the schemas exist: what a Pi device can use stays, the
   Arduino-era parts (ArduinoMother, Wally, Grove, XBee, AquaBubbler...) go to `Avocado/old/`.

### 3.2 Annabelle against the Egg: the parts list to write

Her live Denome (2026-10-31) compared with the Egg, by dene chain. Grouped by the three kinds of §2:

**The kind (parts to define in Avocado):**

| Part | In her Denome today | Avocado |
|---|---|---|
| Annabelle microcontroller (LoRa receiver over the serial port; Annabelle Sensor, Port Status, reset info, Annabelle Actuator) | Components, Sensors, Actuators, Purpose | **done 2026-10-31:** `mcu/Annabelle.mcu`, `hsd/AnnabelleSensor.hsd`, `had/AnnabelleActuator.had` (to build), with the generic generators: `MicroController` (now writes `Async Request Delay Millis`, `Async Execution Mode`, and `Mother Microcontroller` false as well as true), `Sensor` (a boolean value starts `false`, not `0`; any `Initial Value` type; queue position labelled `int`, reporting address `Dene Pointer`), and a new generic `Actuator` (the actuator and an empty `<Actuator> Actions` list that other parts fill). Compared with her live Denome: the microcontroller dene identical. Differences on purpose: her settings list has `ArpScanRetryCount` and `Sample Frequency` (copied from the network inspector; `AnnabelleController` reads neither) and lacks `Serial Data Rate` (the one it reads; 115200, its default): the part has only `Serial Data Rate`; her sensor's value is named `Battery Voltage Value` pointing at a dene `Port Status`, the part uses the convention (`Port Status Value`); her actuator's Codon is labelled `int`. Not in these parts: `Send Graveyard Shift Action` (the Cerebellum's Daffodil task points at it: the Daffodil device-type part), `Settings Update` / `Identity Update` (the SFTP publisher's), reset info and sensor data values (written while she runs). |
| DigitalStables Publisher (controller, Upload/Server/Settings File, two actions, the mutation, Mnemosycon rule) | Components, Actuators, Mnemosycons, Mutations | **done 2026-10-31:** `pub/DigitalStablesPublisher.pub` + `DigitalStablesPublisherHomeoBoxGenerator` (to build). Sertolized and fertilized into the Egg: 11 of her 12 publisher denes identical, and the mutation; the 12th differs on purpose: the rule's `Command` is written as `String`, where hers (and the SFTP and DigitalGeppetto rules) say `int` for a text value; `MnemosyneManager` reads it as a string either way. The first part offered in the palette. |
| SFTP Publisher | Components, Actuators, Mnemosycons | exists: `SFTPPublisher.had` |
| Network Inspector (ArpScanRetryCount, Sample Frequency, Network Status, whitelist actions, Remember Up/Download speed) | Components, Actuators, Sensor Data, Human Interface | exists: `NetworkInspectorMicroController.mcu` + `NetworkSensor.hsd`; check against hers |
| Pi four values on four-digit displays | Actuators | exists: `FourValueFourDigitDisplay.had` (only if her Pi has the displays) |
| Telepathon support (Telepathon View Panel, hourly stale-telepathon delete, Delete Telepathon) | Human Interface, Actuators, Mutations | **done 2026-11-01:** `telepathons/TelepathonSupport.tph` + `TelepathonSupportHomeoBoxGenerator` (to build): the actions `Delete Telepathon Action` (`DeleteTelepathon#<name>`, the name written by the mutation's On Load from the page's payload) and `Hourly Delete Stale Telepathons Action` (`DeleteStaleTelepathons#Hourly`) on the Annabelle Actuator; the mutations `Delete Telepathon` and `Hourly Delete Stale Telepathons` (`Invocation Mode` Time, `Time Mutation Configuration` Hourly); the Home Page's `Telepathon View Panel` and the empty chain `Telepathons View`. The actions are **not** put in `Annabelle Actuator Actions` (that list runs every pulse; in her Denome too only the mutations point at them), so no carrier. Sertolized with Annabelle's microcontroller, sensor, actuator and the publisher, fertilized into the Egg (Fertilizer's check passed) and compared with her live Denome: the hourly action identical, both mutations' flags (Invocation Mode included) identical. Differences on purpose: the Delete command starts `DeleteTelepathon#` (hers carries the name of the last telepathon deleted, garbled by an encoding: what she grew); her panel's `Required` is the text `"true"`; her hourly list's word is named `Delete Telepathon Action` (a copy), the part names it after the action; Fertilizer writes every mutation chain (hers lack the empty injection and On Finish chains). Not here: `Annabelle Queue Status` (the Telepathons nucleus: `MappedBusThread` writes it every pulse, what she grows) and the Cerebellum registry tasks (one per device type: the device-type parts). Needed in Fertilizer: a created mutation takes an optional `Invocation Mode` (to build). |
| Device types she hears (Daffodil, Langley, Chinampa: Cerebellum PulseTask / GraveyardShift / TelepathonRegistryTask) | Internal:Cerebellum | new part per device type, or one part with a list (§9) |
| Memory watch (Heart / Hypothalamus / Web Server resident memory counters) | Actuators, Mnemosyne Pulse Count | new part, probably for every Teleonome: then it belongs in the Egg |
| Database and disk charts, Diagnostics, Mnemosyne page | Human Interface, Analysis | already in the Egg as "Egg ...": renamed at birth |

**This unit (never in the Denome):** her serial number, login, password: the settings file.

**What she grows (never in the Sertoli file):** the telepathons TopTank, Langley_W, Chinampa and their
Mnemosyne entries; Mnemosyne Today data; `Network State Mismatch` pathology; `eth0`/`wlan0 Info`.

**Egg out of date:** Human Interface Control Parameters in the Egg still say `Diagnositcs` and `History`
where Annabelle has `Diagnostics` and `Mnemosyne`. The Egg is brought up to date once, by hand, then
kept current by the same comparison (§7).

## 4. The process: steps, each with its output on screen (Ari, 2026-10-31)

Every step produces a file, and the web page shows it before the next step runs. Nothing is hidden in
between; each output can be opened, read, compared, downloaded.

| # | Step | Input | Output shown |
|---|---|---|---|
| 1 | **Choose the parts** | Avocado's parts | the **Sertoli file** (JSON) |
| 2 | **Sertolize** | Sertoli file + Empty sperm | the **sperm** (JSON), with a summary: homeoboxes, actions |
| 3 | **Fertilize** | sperm + Egg | the **Denome** (JSON), the change against the Egg (added / changed / removed, by dene chain, as in §3.2), Fertilizer's analysis page (`TeleonomeAnalisis.html`), and a **check** (§4.1) |
| 4 | **Birth** | Denome + a name | the named Denome (`Egg` replaced in names and pointers) |
| 5 | **Deliver** | named Denome + a Pi | the Pi's answer (the Denome in place, the Teleonome started) |

Steps 1-4 change nothing outside Avocado's own storage and can be run as often as wanted. Only step 5
touches a Pi, and asks first.

### 4.1 The check after fertilizing

Before birth the Denome is checked, and the problems listed on the page:
- every pointer (`@Egg:...`) points at something that exists;
- every DeneWord of a part's `Configurable` list has a value;
- no two parts claim the same `Processing Queue Position` or the same dene;
- the Denome loads in the framework's `DenomeManager` (the same code the Hypothalamus starts with).

For a Denome being reproduced (§7), also the comparison with the living one.

## 5. The web page

In the factory first (factory2, Tomcat, Bootstrap and jQuery like the rest of the factory), as a new
section **Avocado**. The engine behind it (§6) is a jar of its own, so digitalgeppetto.com can use it
later with its own pages.

```
+-------------------------------------------------------------------------------------------+
| Avocado    Recipe: [Annabelle v]  [New]  [Save]                Egg: Egg.denome (2023)     |
+------------------------------+------------------------------------------------------------+
| PARTS              [search ] | RECIPE: Annabelle                                          |
|                              |                                                            |
| Microcontrollers             |  1. Annabelle microcontroller                [settings][x] |
|   [Annabelle]  [Network      |  2. Network Inspector                        [settings][x] |
|    Inspector]  [Arduino      |  3. DigitalStables Publisher                 [settings][x] |
|    Mother] ...               |  4. SFTP Publisher                           [settings][x] |
| Sensors                      |  5. Telepathon support                       [settings][x] |
|   [Langley] [Network] ...    |                                                            |
| Actuators                    |  (drag parts here; drag to reorder = processing order)     |
|   [SFTP Publisher] ...       |                                                            |
| Publishers                   +------------------------------------------------------------+
|   [DigitalStables] ...       | SETTINGS: DigitalStables Publisher                         |
| Telepathons                  |   Upload To DigitalStables   [x]                          |
|   [Daffodil] [Langley] ...   |   DigitalStables Server      [wss://devices.digit...]      |
| Human Interface              |   Settings File              [/home/pi/Teleonome/devic...] |
|   [Home page panels] ...     |                                                            |
+------------------------------+------------------------------------------------------------+
| 1 Sertoli file [view]  ->  2 Sperm [Sertolize][view]  ->  3 Denome [Fertilize][view][diff] |
|   ->  4 Birth [name: Annabelle ][view]  ->  5 Deliver [host: annabelle-teleonome.local ]   |
+-------------------------------------------------------------------------------------------+
```

- **Parts palette (left).** One card per part file in Avocado, grouped by kind. A card shows the part's
  name and one-line description; hovering shows what it adds (its homeobox summary). A part that needs
  another (the DigitalStables Publisher needs a microcontroller that writes the outbox) says so.
- **Recipe (middle).** Parts are dragged in, reordered by dragging (the order becomes `Processing Queue
  Position`), removed with x. Saving writes the Sertoli file. A recipe can start from an existing one
  ("new Chinampa, starting from Annabelle").
- **Settings.** Built from the part's `Configurable` list (already in the `.mcu` files: name, value
  type, required, default, minimum, maximum). Only settings of the kind; nothing of the unit (no serial
  numbers, logins or passwords: the page refuses fields marked secret or per-unit, §2).
- **The step bar (bottom).** One button per step of §4. Each output opens in a viewer: JSON with
  folding and search (Prism is already loaded by the factory), the diff as a table by dene chain, the
  check's problems as a list linking to the place in the JSON. A step whose input changed shows its old
  output greyed out until it is run again.
- **History.** Every run is kept (recipe version, sperm, Denome, who, when), so "what did we put on
  Annabelle on 2026-11-05" has an answer, and two runs can be compared.

### 5.1 In the factory's product flow

A product definition (Annabelle, Chinampa, Tucci) gets one **recipe**. Preparing a Pi for a product
(today a list of manual steps) becomes: pick the product -> its recipe -> steps 2-5 with the product's
name as the Teleonome's name -> then the factory's existing Send To DS writes the unit's settings
(serial number, login, password). The second Annabelle is: prepare a Pi from the Annabelle recipe, Send
To DS. No hand-edited Denome.

## 6. Changes to Sertoli and Fertilizer

Kept as they are on the command line (a Pi can still sertolize and fertilize itself), and made callable
from a web application:

- **Done for Sertoli (2026-10-31, to build):** `SertoliEngine.sertolize(recipe, emptySperm, avocadoDir,
  name) -> {sperm, errors}`; Sertoli's command line uses it. Lists optional (`MicroControllers` accepted for
  `Components`), a part's folder from its extension, a recipe entry may be `{"Part", "Settings"}` with
  settings overriding the part (`A:B` paths, as in `Configurable`): what the page's settings forms write.
- **A library entry point** next to `main`: `Sertoli.sertolize(sertoliJson, emptySperm, avocadoDir) ->
  sperm` and `Fertilizer.fertilize(sperm, egg) -> denome + analysis`, with no `System.exit`, no paths
  from the working folder, no killing of a Hypothalamus (that stays in `main`, for a Pi). Errors come
  back as a list, not a log line.
- **Birth must accept both spellings (found 2026-10-31):** some generators point at `@Egg:...` (SFTP, DigitalStables
  Publisher, Actuator), others at `@<Teleonome name>:...` (MicroController, Sensor: the name Sertoli is given).
  Birth replaces `Egg` and leaves the rest, so both end as `@Annabelle:`; the generators are to agree on `Egg`.
- **Birth** (new, small): replaces `Egg` with the Teleonome's name in dene names that carry it
  (`Egg Database Pie Chart Data` -> `Annabelle Database Pie Chart Data`) and in every pointer
  (`@Egg:` -> `@Annabelle:`). Also what the rename of 2026-10-30 did by hand on the Pi (hostname,
  tables), as a checklist for step 5. `BirthCertificateCreator` exists in `~/Data/Teleonome` and is to be
  looked at for this.
- **Done for Fertilizer (2026-11-01, to build):** a `Create Mutation` action may carry `Invocation Mode` (`Time` for a mutation the Hypothalamus runs on a schedule); before, every created mutation was run by command only.
- **Fix:** `Template.sperm` (remove the Ra/Cleo example) or use `Empty.sperm`; move or fix
  `FlowMeterEventGenerator.java`; one Egg.
- **New generators** for the new parts of §3.2 (Annabelle microcontroller, DigitalStables Publisher,
  telepathon support, device types). Each generator is a class named after its `Homeobox Definition
  Type`, like the existing ones.
- **Part metadata** for the palette, added to each part file: `Display Name`, `Description`, `Category`,
  `Requires` (other parts), and in `Configurable`: `Secret` / `Per Unit` flags.

These are Teleonome repos: Claude edits and checks with javac; Ari builds, commits, pushes.

## 7. The test: reproduce Annabelle

The design is proven when the recipe `Annabelle.sertoli`, run through steps 2-4, gives a Denome that
matches her living one, except for what she grew (§2 kind 3) and the unit's settings (kind 2, not in
either). The comparison is the one of §3.2, part by part: every difference is either a part still to
write, a part written wrong, or something in her Denome that should not be there (hand-made leftovers).

Only then step 5 on her Pi: her Denome replaced by the reproduced one, with her grown telepathons and
Mnemosyne carried over (to design with Ari: Fertilizer's `Mutate` mode, or birth of a new Denome plus a
copy of what grew).

After Annabelle: **Chinampa** (more sensors, sensor descriptions as `.hsd` parts), then **Tucci**.

## 8. Phases

| # | What | Done when |
|---|---|---|
| 0 | This design agreed; the three Eggs reduced to one, brought up to date | Ari agrees; one `Egg.denome` |
| 1 | Sertoli and Fertilizer as a library (§6), birth, fixes | the Langley run of §3 works from a Java call and the result is named |
| 2 | Annabelle's parts and generators (§3.2) | §7's comparison shows no differences of kind 1 |
| 3 | The Avocado page in the factory: palette, recipe, settings, steps 1-4 with their outputs | Annabelle's recipe made by dragging, its Denome viewed and compared on the page |
| 4 | Step 5 and the product flow (§5.1) | a second Annabelle prepared from the page and Send To DS, no hand edits |
| 5 | Chinampa, Tucci | their recipes reproduce their Denomes |
| 6 | digitalgeppetto.com | the engine jar behind digitalgeppetto's own pages; parts shared, recipes per person |

## 9. Open questions for Ari

1. Device types an Annabelle hears (Daffodil, Langley, Chinampa...): one part per type dragged into her
   recipe, or one *Telepathon support* part with a list of types? (One per type is closer to drag and
   drop, and lets a farm's Annabelle carry only what that farm has.)
2. Is the memory watch (Heart / Hypothalamus / Web Server memory counters) for every Teleonome? Then it
   goes into the Egg, not into a part.
3. Step 5 on a living Teleonome: replace and carry over what grew, or `Mutate` with a sperm of only the
   differences?
4. The Pi's four-digit displays on Annabelle: part of every Annabelle, or only this one?
5. Where the recipes and runs are stored: files in Avocado (git, as today) or the factory database
   (history, who changed what)? Recommended: recipes as files in Avocado (they are design), runs in the
   factory database.
