# <img src="docs/doxygen/BioscaraPanel.png" alt="icon" width="auto" height="25"> Bioscara - DALSA DIY SCARA robot arm
This repository serves to collect all information regarding DALSA's DIY robot arm **Bioscara**.
<p align="center">
  <img src="docs/assets/bioscara_image_1.png" width=500px height=autor />
</p>

## Documentation and Guides
Please refer to the user guides and source code documentation at: https://dalsa-lab.github.io/bioscara/

> [!NOTE]  
> If the website is unavailable, you can find its source code in the *docs/* directory

## Repository Structure
The repository structure is based on the DALSA template.

- ***docs/***: Contains the user-guides as *.md* files and serves as the source code for the Sphinx documentation
- ***hardware/***: Contains the robots CAD model as a STEP file, STL meshes, electrical schematics and further documents
- ***installation/***: Contains installation scripts and files.
- ***lib/***: Contains the source code for the ROS2 workspace and joint firmware.
- ***test/***: Contains system test data and test protocols.


## Branches
There are three core branches which MUST NOT be deleted without appropriate replacement.
- ***main***: This branch contains the latest stable source code. Any commit to this branch will automatically trigger the build of a new documentation.
- ***bioscara_v1***: Contains the original Bioscara project by Karol Garbor.
- ***gh-pages***: contains the HTML source code hosted at https://dalsa-lab.github.io/bioscara/