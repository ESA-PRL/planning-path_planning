<img src="docs/media/DyMuLogo.png" align="right" />

# DyMu
>DyMu (Dynamic Multi-layered) Path Planner results from the ARES (Autonomous Routing on Extreme Surfaces) collaboration activity between the [University of Malaga](https://www.uma.es/robotics-and-mechatronics/info/107542/robotica-espacial/) and the [European Space Agency](https://www.esa.int/Enabling_Support/Space_Engineering_Technology/Planetary_Robotics_Laboratory) under contract 4000118072/16/NL/LvH/gp.

*Author:* [J. Ricardo Sánchez Ibáñez](https://github.com/JRicardoSan) [![orcid](https://orcid.org/sites/default/files/images/orcid_16x16.png)](https://orcid.org/0000-0002-5130-3808) (ricardosan@uma.es)

*Supervisors:* [Carlos J. Pérez del Pulgar](https://github.com/carlibiri) [![orcid](https://orcid.org/sites/default/files/images/orcid_16x16.png)](https://orcid.org/0000-0001-5819-8310) (carlosperez@uma.es), [Martin Azkarate](https://github.com/martinazkarate)(martin.azkarate@esa.int)

## Versions

[![pythonlogo](https://github.com/spaceuma/ARES-DyMu_python/blob/master/docs/media/PythonLogo.jpg)](https://github.com/spaceuma/ARES-DyMu_python)
[![rocklogo](https://github.com/spaceuma/ARES-DyMu_python/blob/master/docs/media/RockLogoLib.png)](https://github.com/esa-prl/planning-path_planning)
[![cpluslogo](https://github.com/spaceuma/ARES-DyMu_python/blob/master/docs/media/CPlusPlusLogo.png)](https://github.com/esa-prl/planning-path_planning/tree/standalone_ver)
[![matlablogo](https://github.com/spaceuma/ARES-DyMu_python/blob/master/docs/media/MatlabLogo.jpg)](https://github.com/spaceuma/ARES-DyMu_matlab)
  
## Reference

In case you use this repository, please cite the following publication:

Sánchez Ibáñez, J. Ricardo, Pérez del Pulgar, Carlos J., Azkarate, M., Gerdes, L., García Cerezo, Alfonso. **Dynamic Path Planning for Reconfigurable Rovers using a Multi-layered Grid**. *Engineering Applications of Artificial Intelligence*, 2019. DOI: [10.1016/j.engappai.2019.08.011](https://doi.org/10.1016/j.engappai.2019.08.011)

```
@article{sanchez2019dynamic,
  title={Dynamic path planning for reconfigurable rovers using a multi-layered grid},
  author={S{\'a}nchez-Ib{\'a}nez, J Ricardo and P{\'e}rez-del-Pulgar, Carlos J and Azkarate, Martin and Gerdes, Levin and Garc{\'\i}a-Cerezo, Alfonso},
  journal={Engineering Applications of Artificial Intelligence},
  volume={86},
  pages={32--42},
  year={2019},
  publisher={Elsevier},
  doi={10.1016/j.engappai.2019.08.011}
}
```
### Media

[Real Experimental Rover using DyMu](https://youtu.be/X4mihNTEVGw)

## C++ Standalone Version guide

### Example Test - Step guide

1. Execute the install script

```
sh install.sh
```

2. Source env.sh

```
. env.sh
```

3. Execute the test

```
runGPPtest
```

4. View the results using python

```
sh viewResults.sh
```

![You should see something like this](https://github.com/esa-prl/planning-path_planning/blob/standalone_ver/docs/media/Example_Results.png)

### Folder structure

| Folder            |       Description                             |
| ----------------- | ------------------------                      |
| data/             | Input data (maps) and saved results           |
| docs/             | Documentation and media resources             |
| src/              | DyMu Source files                             |
| tests/            | Example test file                             |
| utils/            | Contains scripts to display the results       |

### FAQ

- **How can I define the cost to traverse any terrain?**

A look-up table is created in tests/globalPathPlanningTest.cpp. This table provides the cost according to the type of terrain, the locomotion mode and the slope. We assume terrain #0 is non-traversable terrain, i.e. obstacle. However, a high value of cost must still be defined, since a smoothing process is executed and it helps to increase the value of cost of those traversable nodes located next to the obstacles. This is important in order to avoid the extraction of the path in nodes presenting high discontinuities.

- **Where are the required input maps located?**

They are within the data folder. They are essentially two: the DEM (which contains elevation values), and the terrain map (which contains the terrain index for each node).



