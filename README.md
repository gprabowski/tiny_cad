# tinyCAD

## todo

### compulsory
- [ ] deleting points from curve but not from scene
- [ ] change mouse input to position checking this frame instead of callback
- [ ] add a color log
- [ ] is_tag type trait specialization for add/remove component
- [ ] picking by colour in framebuffer

- [ ] add bitset entity list getter
- [ ] add bitset has/can't have entity list getter
- [ ] change maps to sparse sets
- [ ] get rid of passing cm, pass vectors to iterate over instead
- [ ] move camera and rotations to quaternions
- [ ] changing scene / screen coords of cursor
- [ ] consider changing BufaferData to allocation and map + memcpy

### metaprogramming
- [ ] consider use of Select<N, options> instead of long enumeration list
- [ ] std::conditional, doesnt eval inside templates, should not alias them, \
    it also can't take type trait functions as a first argument, one should 
    write a delegate returning bool for such predicate
- [ ] consider common interface via double CRTP on components

### QoL
- [ ] naming of components / systems
- [ ] proper foldering of components / systems
- [ ] add const component getting

### gui
- [ ] modality
- [ ] top menu
- [ ] fonts
- [ ] styling

### nice to have
- [ ] box select
- [ ] axis view with orthogonal projection
- [ ] type erasure for component list storage
