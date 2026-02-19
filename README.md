# Jorge M. Guzmán Portfolio (GitHub Pages)

Static portfolio for robotics, aerospace, and automation internship applications.
Built with plain `HTML + CSS` only.

## Files

- `index.html`: Main landing page (hero, impact, projects, capabilities, experience, education, contact).
- `style.css`: Global styling for all pages.
- `projects/ekf.html`: Featured EKF project page with source-based code snippets.
- `projects/stack.html`: Autonomous navigation stack project page.
- `projects/hpp.html`: Holonomic pure pursuit + pose lock project page.
- `projects/sim.html`: Standalone EKF validation and simulation project page.
- `assets/Jorge_Guzman_Resume.pdf`: Resume file used by all “Download Resume” links.
- `assets/favicon.svg`, `assets/jorge-placeholder.svg`: Site visual placeholders.

## Edit Content

1. Update homepage copy in `index.html`.
2. Update project page details in `projects/*.html`.
3. Keep navigation links consistent across all pages.
4. Keep section IDs (`#projects`, `#capabilities`, etc.) unchanged unless you also update nav links.
5. Keep each project page in the same section order:
`Overview`, `What I Built`, `Technical Notes`, `Results & Validation`, `Code Snippets`.

## Resume PDF

1. Replace `assets/Jorge_Guzman_Resume.pdf` with your real resume file.
2. Keep the same filename to avoid changing links.
3. Use this exact path and casing: `assets/Jorge_Guzman_Resume.pdf`.

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

## HPP / Pose Lock Snippets (projects/hpp.html)

`projects/hpp.html` is structured in the same format as `projects/ekf.html` and uses real
code excerpts for arc geometry, heading logic, and motion-profile integration.

To update these snippets:

1. Open `projects/hpp.html`.
2. Replace only the code inside each snippet card `<pre><code>...</code></pre>`.
3. Paste exact excerpts from your HPP / Pose Lock / profile source files.
4. Keep the 2–4 line explanation under each snippet aligned to what the excerpt literally shows.

Recommended source files to add in this repo for future updates:

- `hpp.hpp` / `hpp.cpp` (or equivalent)
- `pose_lock.hpp` / `pose_lock.cpp` (or equivalent)
- `s-curve-profile.hpp` / `s-curve.cpp` (or equivalent)
- drivetrain interface/controller files used by HPP command output

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
