# tinyCAD

## todo

### compulsory
- [ ] introduce frame state struct with proj, view, lists etc
- [ ] consider common interface via double CRTP on components
- [ ] verify that framebuffer size is used instead of window size
- [ ] deleting points from curve but not from scene
- [ ] change mouse input to position checking this frame instead of callback
- [ ] add a color log
- [ ] is_tag type trait specialization for add/remove component
- [ ] picking by colour in framebuffer

- [ ] add bitset entity list getter
- [ ] add bitset has/can't have entity list getter
- [ ] move to UBO
- [ ] change maps to sparse sets
- [ ] get rid of passing cm, pass vectors to iterate over instead
- [ ] move camera and rotations to quaternions
- [ ] selecting a curve makes all points added be added to that curve
- [ ] moving to geom shader
- [ ] changing scene / screen coords of cursor
- [ ] change all uniforms setting to DSA
- [ ] instead of passing window call glfwGetCurrentContext();

### metaprogramming
- [ ] consider use of Select<N, options> instead of long enumeration list
- [ ] std::conditional, doesnt eval inside templates, should not alias them, \
    it also can't take type trait functions as a first argument, one should 
    write a delegate returning bool for such predicate

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
