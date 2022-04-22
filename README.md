# tinyCAD

## todo

### compulsory
- [ ] move gui to component based
- [ ] cleanup GUI code -> most is the same
- [ ] add a color log
- [ ] is_tag type trait specialization for add/remove component
- [ ] picking by colour in framebuffer
- [ ] system delegate
- [ ] add rectangle with lines on bottom at level 0

- [ ] add bitset entity list getter
- [ ] add bitset has/can't have entity list getter
- [ ] change maps to sparse sets
- [ ] move camera and rotations to quaternions
- [ ] changing scene / screen coords of cursor
- [ ] consider changing BufaferData to allocation and map + memcpy
- [ ] box select

### metaprogramming
- [ ] std::conditional, doesnt eval inside templates, should not alias them, \
    it also can't take type trait functions as a first argument, one should 
    write a delegate returning bool for such predicate
- [ ] consider common interface via double CRTP on components

### QoL
- [ ] naming of components / systems
- [ ] proper foldering of components / systems
- [ ] add const component getting

### gui
- [ ] modality (can be closed and opened via menu)
- [ ] top menu
- [ ] fonts
- [ ] styling
