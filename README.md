# Computer-Pianist
PianoTiles PhysicalAutoPlayer

The motivation for this project lies in the physical and biological limitations of human
reaction time compared to the processing speed of modern microcontrollers. In a typical
rhythm game environment, such as the mobile game Piano Tiles 2, a player is tasked with
identifying a visual stimulus (a black tile appearing on a white background) and reacting by physically tapping the screen.

For a human, this process involves a complex chain of biological events: optical trans-
duction in the retina, signal propagation through the optic nerve, neural processing in the visual cortex, decision-making in the frontal lobe, and finally, the propagation of motor signals to the muscles in the fingers. The average human reaction time to a visual stimulus is approximately 250 milliseconds. While this is suﬃcient for casual gameplay, it imposes a hard ”speed limit” on performance. Physically, this limits a human player to a maximum frequency of approximately 4–5 Hz (notes per second). As game diﬃculty increases, note density often exceeds this biological bandwidth, making perfect performance physiologically impossible.

This project seeks to overcome these biological constraints by replacing the human neural
pathway with an electronic circuit and software logic. By utilizing a microprocessor (Ar-
duino Uno) operating at a clock speed of 16 MHz, the ”reaction time” can be reduced from
hundreds of milliseconds to mere microseconds. Unlike human nerve impulses, which travel
at approximately 100 m/s, the internal signals of the microcontroller propagate at a significant fraction of the speed of light. This project explores the physics of optoelectronics and electromechanics to create a closed-loop system capable of playing Piano Tiles at speeds that are strictly impossible for a human player, limited only by the physical articulation time of the actuators.
