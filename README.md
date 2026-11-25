# LEGO Autonomous Robot — Public Showcase

This repository is the **public showcase** of the LEGO Autonomous Robot project:

- the interactive page and demo media;
- the architecture, algorithm, simulation and validation notes;
- no source code.

**Live site:** https://sejotta.github.io/lego-autonomous-robot/

The full Python source code, tests, configuration and robot-specific calibration live in a **private repository**. This public repository exists so people can see what the project is and how it works, without publishing the code.

---

## What is public

| Path | Content |
|---|---|
| `index.html` | The integration and technical showcase page |
| `docs/ARCHITECTURE.md` | Layered architecture and module boundaries |
| `docs/ALGORITHMS.md` | NF1, odometry, visual servoing and ORB/RANSAC |
| `docs/SIMULATION.md` | Simulation process and sim-to-real gaps |
| `docs/VALIDATION.md` | What was tested, what was fixed and what remains hardware-dependent |
| `docs/gifs/` | Real robot recordings, optimized |
| `docs/images/` | Plans, trajectories and vision figures |

## What is private

- `robot/` modules and the mission entry points;
- tests and hardware adapters;
- maps/configuration used for the physical runs;
- the full source tree and its history.

## Request access to the code

The code is shared case by case. Write to **sertor2004@gmail.com** with:

- your GitHub username;
- the robot/hardware you are using;
- what you want to do with the code;
- your deadline, if you have one.

Suggested subject:

```text
Solicitud de acceso al código del robot · [tu universidad]
```

I will reply with the conditions and, if it makes sense, grant access to the private repository.

> The email is already present in the public commit metadata. If you prefer not to publish it, replace it in `index.html` with a contact form or a GitHub issue link.

---

## Run the page locally

Open `index.html` directly, or serve the folder:

```bash
python -m http.server 8000
```

Then open `http://localhost:8000/`.

The page uses only local assets; no CDN, analytics or external JavaScript is required.

---

## Credits

Project by **Sergio Toro Codo** and **Guillén Sainctavit**, for the **Robots Autónomos** course at **EINA, Universidad de Zaragoza**, with the guidance and correction of the teaching team.

R2-D2 and BB-8 are trademarks of their respective owners. This is not an official publication of the University of Zaragoza.
