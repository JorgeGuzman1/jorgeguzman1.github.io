# Jorge M. Guzman Portfolio (GitHub Pages)

Static portfolio for robotics, aerospace, and automation internship applications.
Built with plain `HTML + CSS` only.

## Files

- `index.html`: Main landing page (hero, impact, projects, capabilities, experience, education, contact).
- `style.css`: Global styling for all pages.
- `projects/ekf.html`: Featured EKF project page with source-based code snippets.
- `projects/stack.html`: Autonomous navigation stack project page.
- `projects/hpp.html`: Holonomic pure pursuit + pose lock project page.
- `projects/sim.html`: Simulation and verification project page.
- `assets/Jorge_Guzman_Resume.pdf`: Resume file used by all “Download Resume” links.
- `assets/favicon.svg`, `assets/jorge-placeholder.svg`: Site visual placeholders.

## Edit Content

1. Update homepage copy in `index.html`.
2. Update project page details in `projects/*.html`.
3. Keep navigation links consistent across all pages.
4. Keep section IDs (`#projects`, `#capabilities`, etc.) unchanged unless you also update nav links.

## Resume PDF

1. Replace `assets/Jorge_Guzman_Resume.pdf` with your real resume file.
2. Keep the same filename to avoid changing links.

## EKF Snippets (projects/ekf.html)

`projects/ekf.html` contains excerpt cards for:

- Holonomic prediction Jacobian
- Covariance propagation
- 2D GPS Joseph update
- `mat3_mul`
- `SensorFeeder::step`
- `readGpsXY` RMS gating

To expand or refresh snippets:

1. Open `projects/ekf.html`.
2. Update the corresponding snippet card `<pre><code>...</code></pre>` blocks.
3. Paste exact excerpts from your source files without rewriting logic.
4. Keep explanations short and limited to behavior visible in the snippet.

If you keep source files in this repo, recommended names are:

- `EKF.hpp`
- `EKF.cpp`
- `sensor_feeder.hpp`
- `Sensor_Feeder.cpp`

## GitHub Pages Deployment

This repo is a user site (`jorgeguzman.github.io`), so GitHub Pages serves from the root of the default branch.

1. Commit changes.
2. Push to `main`.
3. Wait 1 to 3 minutes.
4. Open `https://jorgeguzman.github.io/`.

If needed, verify in GitHub:

- `Settings` -> `Pages`
- Source: `Deploy from a branch`
- Branch: `main`
- Folder: `/ (root)`

## Local Preview

Run from repo root:

```bash
python3 -m http.server 8080
```

Open `http://localhost:8080`.
