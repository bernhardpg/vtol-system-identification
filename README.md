# VTOL UAV System Identification

**If you use any of this, please cite the original thesis:**
> B. P. Graesdal, “Full Nonlinear System Identification for a Vertical-Takeoff-and-Landing Unmanned Aerial Vehicle,” Master thesis, NTNU, 2021. Available: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2981320

```bibtex
@mastersthesis{graesdal2021vtol,
  author = {Graesdal, Bernhard Paus},
  title  = {Full Nonlinear System Identification for a Vertical-Takeoff-and-Landing Unmanned Aerial Vehicle},
  school = {Norwegian University of Science and Technology (NTNU)},
  year   = {2021},
  url    = {https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2981320}
}
```

**If you are looking for the Babyshark 260 VTOL UAV model, it is implemented here**: [Dynamic Model of the Foxtech Babyshark 260 VTOL UAV](https://github.com/bernhardpg/babyshark_vtol_model)

This repo is made publically available for future reference for work on system identification or further modelling of the Babyshark VTOL. This repo is not written to be used as a library in itself, although it does provide implementations of all the methods used in the system identification of the Babyshark VTOL UAV.

## License
The code (`.m`, `.py`, `.c` and `.js` files) is released under the [MIT License](LICENSE).

All other files, including the flight data, the AVL model, the 3D model and the thesis, are released under the [Creative Commons Attribution 4.0 International License (CC BY 4.0)](LICENSE-CC-BY-4.0). You may use them for any purpose, including commercially, as long as you credit this work by citing:
> B. P. Graesdal, “Full Nonlinear System Identification for a Vertical-Takeoff-and-Landing Unmanned Aerial Vehicle,” Master thesis, NTNU, 2021. Available: https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2981320

The following third-party files are not covered by either license and keep their original terms:
* `avl_model/avl_model/avl3.35` and everything in `avl_model/avl_help/`: the AVL program and its documentation, by Mark Drela and Harold Youngren.
* `avl_model/avl_model/e397.dat` and `avl_model/avl_help/example/sd7037.dat`: airfoil coordinates.
