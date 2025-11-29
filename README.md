# lego-gantry

## Background

My father previously made a [MOC](https://www.bricksmcgee.com/glossary/moc/) that moved a UFO in a circle in the air using a gantry: the UFO was controlled via four spools of fishing line, each controlled by four [Mindstorms NXT](https://www.bricksmcgee.com/glossary/mindstorms/) bricks. The NXT bricks communicated with each other via bluetooth, and rotated the spools using their own respective motors.

This year, we were approached by other [AFOLs](https://www.bricksmcgee.com/glossary/afol/) to collaborate on an extension to an existing [GBC (Great Ball Contraption)](https://www.greatballcontraption.com/) module, which is styled as a sci-fi monorail. The goal is to create a spaceship that "flies" (using a gantry system like my father's MOC) to playfully disrupt the monorail, while still maintaining the robustness and throughput requirements of the GBC module.

## Purpose of this repo

This GitHub repo is specifically for the development of the Gantry. While there is some CAD for Lego (e.g. [studio](https://www.bricklink.com/v3/studio/main.page)), the main reason why we want to use GitHub's version control and issue tracking is the gantry control code. The new controls should be able to handle much more than a flat circular path.

I started by developing in this [Google Colab project](https://colab.research.google.com/drive/1Y4Q8gQNRaKeyEFv5qU9beZ7aUS5cybl_?usp=sharing), but we realized we had more ideas than I could easily track there.
