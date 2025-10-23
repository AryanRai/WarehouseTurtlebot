# MTRX3760 - Project 2  
## Warehouse Robot DevKit  

**Weight:** 20% of final mark  
**Total Marks:** 100  

Presentations will be held during your **lab session** in **Week 12 (Fri labs)** and **Week 13 (Mon–Weds labs)**.  
**Reports** are due **Friday of Week 13 by 23:55**.  

Late submissions will follow the University’s late policy. Plagiarism will be handled according to University policy.  
No submissions will be accepted more than **one week after the due date**.

---

## 🧑‍🤝‍🧑 Group Work

- Complete this project in **groups of 5 or 6** students.  
- Each group must nominate **one leader** who will submit the project to Canvas.  
  - Only one submission per group.  
- All group members must be in the **same timetabled lab section** — *this is not negotiable.*  
- Larger groups will be graded with **higher expectations**.

---

## 🧾 Submission Components

Your submission will be assessed on the following components.  
**Incomplete submissions will not be graded.**

### 1. Presentation and Live Demo
- Present your project **in-person** to your peers.  
- Include a **brief talk/slide deck** and a **live demo** of your system’s capabilities.  
- All team members **must speak** during the presentation.

### 2. Report
- Submit a **single PDF report** via Canvas.  
- Keep it **succinct**, directly addressing the assignment questions.  
- The **front page** should include:
  - SIDs and tutorial sections of all members  
  - **No names**, to comply with anonymous marking  
- The **appendix** must contain a **printout of all C++ code** (in text format).  
  - Include **file header comments** and proper formatting.  
  - Use the **mandatory file header format** (see below).

### 3. Code .zip File
- Submit a **.zip file** of all your **C++ code** via Canvas.  
- Do **not include binaries**.

---

## 🏗️ Background

The **warehouse robotics sector** in Australia is rapidly growing due to e-commerce expansion and automation demand.

- **2022:** DHL invested **AUD $150M** to deploy **1,000 robots** in fulfilment centres.  
- **2024:** Coles launched its **87,000 m² automated distribution centre** in Western Sydney, processing over **10,000 orders daily**.  
- **2023:** The Australian warehouse robotics market generated **AUD $256M**, projected to reach **AUD $981M by 2030**.

These developments highlight the need for **versatile, adaptable robotic systems**.

**Reference links:**
- [DHL Supply Chain invests $150 million in warehouse robotics](https://www.dhl.com/au-en/home/press/press-archive/2022/dhl-supply-chain-invests-150-million-in-warehouse-robotics-across-australian-fulfilment-centres.html)
- [Coles unveils distribution centre](https://www.colesgroup.com.au/news/2024/media-releases/?page=coles-unveils-state-of-the-art-distribution--centre-in-western-sydney)
- [Grandview Research: Australia Warehouse Robotics Market](https://www.grandviewresearch.com/horizon/outlook/warehouse-robotics-market/australia)

---

## ⚙️ The Heterogeneous Platform Problem

Warehouses use **multiple types of robots** due to:
- Cost efficiency via specialised platforms  
- Gradual replacement of old fleets  

This creates **heterogeneity challenges**:
- Difficult to write code for each robot type  
- Hard to reuse code across platforms  

### 💡 The Approach
To solve this:
- Use a **common development platform** that can **emulate multiple robots**
- Write **polymorphic C++ code** for shared functionalities  
- Demonstrate a **principled, extensible, maintainable design**

Your job:  
**Pitch a design approach** to investors that shows your system can emulate multiple robot types on one platform.

---

## 📋 Project Requirements

### 🧠 Common Platform
- Use **one physical Turtlebot 3**.  
- **No hardware modifications** (camera placement, sensors, etc.)  
- **Do not damage** the Turtlebot.

### 🔧 Core Functionality
Emulate **at least two types of robot**:

#### 1. Inspection Robot
- Has **camera + LiDAR + odometry**  
- Locates **damages in the warehouse**  
- Saves damage records to disk (text format)

#### 2. Delivery Robot
- Uses **LiDAR + odometry** (no camera)  
- Delivers goods between locations  
- Handles **multiple delivery requests**  
- Saves delivery records to disk (text format)

### 🧱 Shared Functionality
Both robots must include:
- Basic **motion control** (differential drive)  
- Virtual **battery level tracking and communication**

---

## 🌟 Extensions (Optional)

To achieve an **HD (High Distinction)**, implement all core functionality **plus at least two** of the following (or similar complexity):

- 🗺️ Autonomous mapping of the warehouse  
- 📍 Navigate to a selected damage site  
- ⚡ Auto-dock at a charging station when low on battery  
- 📦 Intelligent delivery scheduling to maximise throughput  
- 🤝 Collaborative operation with another team’s robot (e.g., collision avoidance)

---

## 💻 Implementation

- You may use **ROS packages**, **libraries**, and **open-source code**.  
- **Substantial functionality must be written by you in object-oriented C++.**  
- Python or other languages can be used to **bind** components (e.g., camera),  
  but **no marks** are given for non-C++ parts.  

---

## 🧪 Testing / Demonstration

- You decide how to test and demonstrate your system.  
- Collaboration with other teams is allowed.  
- To receive marks, all functionality must be **shown in the live demo**.  

**Tips:**
- Use **coloured patches**, **AprilTags**, or **ArUco markers** for easier visual detection.  
- You don’t need dynamic switching between inspection and delivery modes.  
- Be **courteous** in the shared lab — clean up after use.

---

## 🧩 Deliverables

You’ll be assessed on:
- Application of **project management**, **system design**, and **coding** principles.  
- Refer to the **rubric** at the end.

---

## 🎤 Presentation and Demonstration

- Max **25 minutes** + **5 minutes Q&A**.  
- Combine **slides** and **live robot demo**.  

### Presentation Must Include:
- Team introduction and roles  
- System functionality and motivation  
- Top-level system design overview  
  - Use diagrams (e.g., ROS Node and UML Class Diagrams)  

**Tips:**
- Avoid small/illegible text on slides  
- All team members must present and attend  
- Attend other groups’ presentations  
- Practise using the projector beforehand  

---

## 🧾 Report

Keep the report **brief** — no demo results needed (those are shown live).  
Focus on **object-oriented design** and **coding principles**.

### 📘 Sections

#### System Design
- Bullet-point list of achieved functionalities  
- Diagrams showing system modules and ROS nodes  
- Indicate which processor each node runs on  

#### Teamwork & Project Management
- Include git commit history via:
  ```bash
  git log --graph --pretty=format:"%h %ad %an %s" --date=short
