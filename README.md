<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->

<!-- PROJECT LOGO -->
<br />
  <div align="center">
    <a href="https://roboticsmick.pythonanywhere.com/">
      <img src="https://roboticsmick.pythonanywhere.com/assets/LSSRlogo.png" alt="LSSR Logo" height="300">
    </a>
  <h3 align="center"></h3>

  <h3 align="center">LSR Rocket Avionics</h3>

  <p align="center">
    An open source rocket avionics system to track and monitor rocket flight using the RP2040 MCU.
    <br />
    <a href="https://github.com/roboticsmick/LSSR_RA">View Demo</a>
    ·
    <a href="https://github.com/roboticsmick/LSSR_RA/issues">Report Bug</a>
    ·
    <a href="https://github.com/roboticsmick/LSSR_RA/issues">Request Feature</a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project

GPS, accelerometer, barometric and gyoscope data is logged during the rockets flight. 

This data is also transmitted back to a ground station and can be used to locate the rocket, and model it's flight path. 

Barometric and accelerometer sensors can also be used to deploy the parachute ejection charges at the optimal time.

![GPS_tracker](https://user-images.githubusercontent.com/70121687/205568776-9605248e-3c44-4a2d-9448-0044c5223a49.png)
![GPS](https://user-images.githubusercontent.com/70121687/205567675-e1951953-1021-4b9b-8e1e-82c31342c12b.png)
![plotGPS](https://user-images.githubusercontent.com/70121687/205567726-5fbc1030-31a9-4e93-8f6d-4dfd76b16a25.png)


<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

[![CPP][cpp]][cpp-url]
[![pi]][pi-url]
[![vsc]][vsc-url]
[![plotly]][plotly-url]
[![pythonanywhere]][pythonanywhere-url]
[![patreon]][patreon-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.

  ```sh

  WORK IN PROGRESS

  ```

### Installation

Installation instructions will be added when I am a little more confident this will work. Use at your own risk.

1. Clone the repo

   ```sh

   git clone https://github.com/roboticsmick/LSR_RA.git

   ```

2. Put the pico into bootloader mode
3. Build the LSR Rocket Avionics code.
4. Copy the UF2
  
   ```sh

   cp -a ./build/LSR_RA.uf2 /media/logic/RPI-RP2/

   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://michaelvenz.com/)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

* [ ] Fix CMakefiles and folder structure so they are setup correctly.
* [ ] Make sure setup correctly for C and C++
* [ ] Add drivers for all sensors.
  * [ ] Configure drivers to work with SPI and I2C
  * [ ] Add clear instructions without jargon for all drivers
  * [ ] Add pinout instructions for all drivers
* [ ] Redo in Zephyr RTOS

See the [open issues](https://github.com/roboticsmick/LSR_RA/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

Please reach out if you have any suggestions or want to help make this system better. I'm a beginner and learning how to build this in my spare time so I'd love some help.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

* Michael Venz: [michaelvenz.com](https://michaelvenz.com/)
* Github: [Github](https://github.com/roboticsmick/)
* Twitter: [@roboticsmick](https://twitter.com/roboticsmick)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

Use this space to list resources you find helpful and would like to give credit to. I've included a few of my favorites to kick things off!

* [Shawn Hymel - Intro to Raspberry Pi Pico and RP2040](https://www.youtube.com/playlist?list=PLEBQazB0HUyQO6rJxKr2umPCgmfAU-cqR)
* [Othneil Drew - Best README Template](https://github.com/othneildrew/Best-README-Template)
* [Ileriayo Adebiyi - Shield Icons ](https://github.com/progfay/shields-with-icon)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/roboticsmick/LSR_RA.svg?style=for-the-badge
[contributors-url]: https://github.com/roboticsmick/LSR_RA/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/oroboticsmick/LSR_RA.svg?style=for-the-badge
[forks-url]: https://github.com/roboticsmick/LSR_RA/network/members
[stars-shield]: https://img.shields.io/github/stars/roboticsmick/LSR_RA.svg?style=for-the-badge
[stars-url]: https://github.com/roboticsmick/LSR_RA/stargazers
[issues-shield]: https://img.shields.io/github/issues/roboticsmick/LSR_RA.svg?style=for-the-badge
[issues-url]: https://github.com/roboticsmick/LSR_RA/issues
[license-shield]: https://img.shields.io/github/license/roboticsmick/LSR_RA.svg?style=for-the-badge
[license-url]: https://github.com/roboticsmick/LSR_RA/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/roboticsmick/
[product-screenshot]: images/screenshot.png
[cpp]: https://img.shields.io/badge/C/C++-black.svg?style=for-the-badge&logo=C%2B%2B&logoColor=wh
[cpp-url]: https://michaelvenz.com/
[pi]:https://img.shields.io/badge/-Raspberry%20Pi%20Pico-C51A4A?style=for-the-badge&logo=Raspberry-Pi
[pi-url]: https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html
[vsc]:https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white
[vsc-url]: https://code.visualstudio.com/download
[plotly]:https://img.shields.io/badge/Dash%20Plotly-%233F4F75.svg?style=for-the-badge&logo=plotly&logoColor=white
[plotly-url]: https://dash.plotly.com/introduction
[pythonanywhere]:https://img.shields.io/badge/pythonanywhere-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54
[pythonanywhere-url]: https://roboticsmick.pythonanywhere.com/
[patreon]:https://img.shields.io/badge/Patreon-F96854?style=for-the-badge&logo=patreon&logoColor=white
[patreon-url]: patreon.com/user?u=64698997
