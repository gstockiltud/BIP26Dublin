# STMP26 SENSATE: Short Term Mobility Program 2026
## International Summer School — TU Dublin Tallaght Campus

`#STMP26` `#SENSATE` `#TUDublin` `#EUTplus` `#GNSS` `#Arduino` `#ESP32S3` `#OSNMA`

---

![SENSATE International Summer School Banner](https://raw.githubusercontent.com/gstockiltud/BIP26Dublin/main/images/bannerSTMP26_6unis.png "SENSATE 2026")

> 💡 **Program Nomenclature Note:** To align with European funding streams, administrative frameworks, and institutional requirements, this initiative has transitioned from its initial working title of *BIP26* to its official designation: **STMP26 SENSATE (Short Term Mobility Program)**.

---

## 📋 Program Overview

Welcome to the central repository for **STMP26 SENSATE** (**SENsors and Software for Applications and Technological Education**). This intensive international summer school brought together over **55 engineering students from 6 partner countries** for an immersive, hands-on experience in advanced hardware prototyping, sensor fusion, and secure satellite communication.

Hosted at the **Technological University Dublin (Tallaght Campus)** from **June 29 to July 4, 2026**, the program bridged academic theory and cutting-edge industry practice through collaborative, AI-assisted rapid prototyping and hardware engineering.

---

## 🔍 Explore the Program Reference Sections

Click on any section header below to expand and view the detailed acknowledgments, curriculum metrics, and codebase references.

<details>
<summary><b>🏛️ 1. Institutional Support, Funding & Industry Sponsorship</b></summary>
<br>

The STMP course was developed by the School of Electronic and Electrical Engineering at TU Dublin, in close collaboration with our European university partners and with the valued assistance of colleagues from across other academic departments. The program was proudly funded by EUT+ (European University of Technology) and sponsored by Tokyo Electron (TEL).

The incredible success of STMP26 SENSATE was made possible through robust European institutional backing and high-tech industry alignment:
* **European University of Technology (EUT+):** Providing the foundational cross-continental integration framework for academic alignment and standardized student mobility.
* **Erasmus+ Short Term Mobility Framework:** Financing and administering the complex logistics required to enable multi-lateral student exchanges.
* **Tokyo Electron (TEL):** Special thanks to **Joanna** and the corporate team at TEL for industry sponsorship, helping foster deep semiconductor industry awareness and supporting the next generation of electronics and systems engineers.
</details>

<details>
<summary><b>🏭 2. Thanks to Tokyo Electron for Industry Link and Sponsorship</b></summary>
<br>

A very special thank you to TEL (Tokyo Electron) and their training facility in Santry, which our students were the first university group to visit. There they learned first-hand just how important the makers of semiconductor equipment are to modern life.

Tokyo Electron is one of the world's largest suppliers of the precision machinery used to manufacture microchips — the equipment that etches, coats, cleans and processes the silicon wafers at the heart of every phone, computer and satellite receiver. In short, TEL builds the machines that make the chips that power modern life.

TEL is also a genuinely global company, with sites right across Europe, Asia and the Americas — and it actively seeks talented engineers from all disciplines, not just electronics. Mechanical, software, materials, chemical and systems engineers all come together in creative, multidisciplinary teams — exactly the kind of teamwork our students practised this week. For a young engineer, that opens doors all over the world.

Through its "TEL for Good" corporate social responsibility programme, the company also invests in education, community and sustainability — supporting STEM learning and the next generation of engineers, which is very much the spirit behind their support for our summer school.

Special thanks to **Giedre Simkute**, **Joanna Orlinska**, and all at TEL.
</details>

<details>
<summary><b>🌍 3. Program History, Heritage & Regional Expansion (Special Tributes)</b></summary>
<br>

SENSATE has established a prestigious pedigree across Europe, building upon consecutive successful international iterations. We proudly recognize the legacy of our partner network:
* **The Heritage Track:** Prior highly successful editions were hosted and developed in collaboration with:
  * **Valencia (2023, Polytechnic University of Valencia, Spain):** Thanks to Professor Sara Blanc Clavero for getting this program started. Your students are very lucky to have you.
  * **Cluj-Napoca (2024, Technical University of Cluj-Napoca, Romania):** Thanks to Professor Laura Grindei for getting this program started. Your students are very lucky to have you.
  * **Darmstadt (2025, Hochschule Darmstadt, Germany):** With special thanks to Professor Eiken Lübbers for adding the space theme to SENSATE in 2026 with the exciting rocket launches.
* **New Network Additions & Thanks:** We extend a warm, official welcome and deep gratitude to our new consortium institutional partners joining the mobility network this year:
  * **FH Campus Wien (Austria):** Thanks to **Thomas Fischer** for his outstanding contribution in Dublin and the excellent students of **Advanced Manufacturing Technology**.
  * **University of Cassino (Italy):** Thanks to **Prof. Emanuele Grossi** and the excellent students in **Telecommunications** who had a natural understanding of satellite signals.
</details>

<details>
<summary><b>🚀 4. Technical Curriculum Outline & Sample Student Achievements</b></summary>
<br>

The curriculum challenged students to use advanced embedded architecture, real-time feedback loops, anti-spoofing protections, and telemetry. What tied this together was a dynamic framework of 6-nation teamwork. 

Some selected core curriculum topics include:
* **Microcontroller Prototyping:** Core development revolved around the high-performance **Tenstar TS-ESP32-S3** dual-core processor using the **Arduino IDE** environment.
* **Visual Telemetry & Feedback:** Low-level implementation of interactive graphic layouts using **ST7789 TFT Displays**, alongside sophisticated web displays using SQL, and integrated touch screen interfaces.
* **Advanced Satellite Navigation (GNSS):** Interfacing with an advanced **GNSS Multiband Receiver** to handle live multi-constellation positional feeds. Students used these streams to track paths, map geographical contours, interface with magnetometers, and even develop interactive games.
* **Security & Resilience Focus:** A core lecture and lab framework dedicated to **Galileo Open Service Navigation Message Authentication (OSNMA)**, equipping students to understand, analyze, and build protection against real-world signal spoofing and jamming vulnerabilities.
* **AI-Assisted Pedagogy with 3D Printing and GitHub:** Active integration of Generative AI workflows for localized code refactoring, rapid diagnostic debugging, and system-level library integration. Extensive use was made of **GitHub** for codebase tracking and **3D Printing** for structural prototyping. Our students were amazingly productive working across their multi-disciplinary teams!
</details>

<details>
<summary><b>💻 5. Repository Codebase & Core Firmware Applications</b></summary>
<br>

This repository hosts production-ready firmware skeletons, diagnostic tools, and complete student reference applications located in the primary source folders:
* 📁 `NMEA_Read/NMEA_Read.ino`: The baseline diagnostic routine designed to establish a raw serial link with the Waveshare GNSS hardware, parsing standard NMEA sentences and printing them to the local terminal.
* 📁 `STMP26_SkyView/STMP26_SkyView.ino`: An interactive graphical application visualizing live satellite tracking constellations, active azimuths, and signal strength metrics (SNR) in real-time.
* 📁 `STMP26_LiveDashboard/STMP26_LiveDashboard.ino`: The master firmware suite integrating the TS-ESP32-S3 processing core with the ST7789 TFT layout and local NeoPixel status indicators, establishing a fully self-contained telemetry monitoring terminal.
</details>

<details>
<summary><b>☘️ 6. Host Institution & Logistical Support Pillars (Thank You!)</b></summary>
<br>

A unified welcome and massive thank you is extended to all international delegates from across all campuses of **Technological University Dublin**. This edition was centrally anchored at the **Tallaght Campus**, utilizing state-of-the-art engineering laboratories and collaborative spaces.

### 🌟 Key Acknowledgments
The complex organizational architecture, student housing logistics, hardware procurement, and administrative planning would not have been possible without the tireless, dedicated efforts of our core coordination team. 

While it is impossible to thank individually every person who contributed across all participating campuses and our six partner nations, the following two individuals truly went the extra mile (or two). They beautifully represent the exceptional spirit of dedication and collaborative teamwork that defines our community:
* **Susanne Murphy** — Academic & Administrative Coordination
* **Dave Maguire** — Technical & Logistical Infrastructure Support

Thank you for making STMP26 SENSATE at the Tallaght Campus, Dublin, between 29th June and 4th July, an unforgettable and positive experience for many! We are delighted to follow in the tradition of previous outstanding successes in Valencia, Cluj-Napoca, and Darmstadt.
</details>

---

## 🛠️ Quick Start Guide

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/gstockiltud/BIP26Dublin.git](https://github.com/gstockiltud/BIP26Dublin.git)
