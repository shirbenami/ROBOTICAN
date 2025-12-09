# 🚀 Distributed Drone Vision and Planning System README

This document serves as the operational manual for the distributed drone vision and planning system. It outlines the network configuration, component distribution, and required startup sequence.

---

## 1. ⚙️ System IP and Device Mapping

The system operates exclusively on the **192.168.131.X** subnet.

| Device             | Primary IP Address                   | Hostname/Purpose | User Name | Notes                               |
|:-------------------|:-------------------------------------| :--- | :--- |:------------------------------------|
| **Drone (R1)**     | `192.168.131.11` / `192.168.131.101` | Remote Vehicle | N/A | Two potential IPs for R1            |
| **Drone (R2)**     | `192.168.131.12` / `192.168.131.102` | Remote Vehicle | N/A | Two potential IPs for R2            |
| **Drone (R3)**     | `192.168.131.13` / `192.168.131.103` | Remote Vehicle | N/A | Two potential IPs for R3            |
| **Comm Unit**      | `192.168.131.19`                     | Communication Hub | N/A |                                     |
| **Jetson Nano**    | `192.168.131.21`                     | **LLM Converter** | `user` | Runs command conversion             |
| **AGX 1**          | `192.168.131.22`                     | **Vision & Comm** | `user` | Primary vision and state management |
| **AGX 2**          | `192.168.131.23`                     | **LLM Planner** | `nvidia` | Runs the main planning logic        |
| **Laptop/Backend** | `192.168.131.24`                     | **Backend Docker** | `user` | Controls streaming and movement     |

---

## 2. 🗺️ Functional Block Diagram

The diagram below illustrates the high-level data flow and the distribution of software components across the physical hardware.



---

## 3. 🖥️ Component Allocation and Software

This section details the specific software components that must be running on each designated device.

### 3.1. Jetson Nano (`192.168.131.21`)

* **User Name:** `user`
* **Primary Function:** **LLM Command Pre-processing**
* **Component:**
    * **LLM Converter:** Converts user input (e.g., natural language command) into a structured request suitable for the LLM Planner.
* **How to run?**
  * see **NanoLLM_VILA_and_OWL** README file

### 3.2. AGX 1 (F) (`192.168.131.22`)

* **User Name:** `user`
* **Primary Function:** **Vision Processing & Communication Management**
* **Components:**
    * **VILA (Vision-Language Model Inference):** Executes the VLM for high-level image understanding.
    * **OWL (Object Detection):** Performs specialized object recognition and localization.
    * **comm\_manager\_2:** Handles the primary communication protocols.
    * **image\_state\_subscriber\_call\_vlm:** Subscribes to image data, prepares it, and triggers the VILA/OWL vision tasks.
* **How to run?** 
  * **VILA**, **OWL**, **comm_2** - see **NanoLLM_VILA_and_OWL** README file
  * **image\_state\_subscriber\_call\_vlm:** 
```bash

cd rqs_iai_ws/
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/examples/src/video_and_capture/
python3 image_state_subscriber_call_vlm.py --out-dir /home/user/jetson-containers/data/R2 --vlm http://192.168.131.22:8080/describe
```


### 3.3. AGX 2 (No Sticker) (`192.168.131.23`)

* **User Name:** `nvidia`
* **Primary Function:** **Action Planning & Decision Making**
* **Component:**
    * **LLM Planner:** Receives the converted command and processed vision data to generate a sequence of low-level **actions/movements** for the Drone.

### 3.4. Laptop/Backend (`192.168.131.24`)

* **User Name:** `user`
* **Primary Function:** **Drone Control & Data Ingress** (Running inside Docker)
* **Components:**
    * **video\_stream.py:** Manages the incoming live video feed from the Drone.
    * **Capture Frames Logic:** Extracts and transmits necessary video frames to **AGX 1**.
    * **Movement Control Module:** Receives movement commands from **AGX 2** and executes them on the Drone.
* **How to run?**
  * **open the backend of the sphera**
```bash
docker-compose up it
```

in another terminal:

```bash

docker attach it
pip3 install opencv-python
sudo apt install ros-foxy-cv-bridge -y
cd ~/workspace/src/examples/src/video_and_capture
python3 video_stream.py 
```
in another terminal capture frame when arriving an angle:
```bash 

docker exec -it it bash
cd ~/workspace/src/examples/src/video_and_capture
python3 azimuth_capture_client.py 
```
* missing: movement handler
---

## 4. ▶️ Startup Sequence (Recommended)

Follow these steps precisely to ensure all components start in the correct order to resolve dependencies and ensure proper communication.

1.  **System Preparation:**
    * Power on all devices (Drone, Comm Unit, Jetson Nano, AGX 1, AGX 2, Laptop).
    * Verify network connectivity by pinging all devices in the `192.168.131.X` range.

2.  **Start Vision/Communication Managers (AGX 1):**
    * SSH into $\mathbf{AGX\ 1\ (192.168.131.22)}$ as `user`.
    * Start the $\mathbf{VILA}$, $\mathbf{OWL}$, $\mathbf{comm\_manager\_2}$ and $\mathbf{image\_state\_subscriber\_call\_vlm}$ services.

3.  **Start Backend Control (Laptop/Backend):**
    * Log into the **Laptop ($\mathbf{192.168.131.24}$) as `user`** and enter the Docker environment.
    * Execute $\mathbf{video\_stream.py}$ to initiate the video feed.
    * Start the **Capture Frames Logic** to push images to AGX 1.
    * Start the **Movement Control Module** (listening for commands from AGX 2).

4.  **Start LLM Components:**
    * SSH into $\mathbf{Jetson\ Nano\ (192.168.131.21)}$ as `user`.
    * Start the **LLM Converter** service.
    * SSH into $\mathbf{AGX\ 2\ (192.168.131.23)}$ as `nvidia`.
    * Start the **LLM Planner** service.

The system is now operational and awaiting commands via the **LLM Converter** on the Jetson Nano.