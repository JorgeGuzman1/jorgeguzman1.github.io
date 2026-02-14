# Jorge Guzmán Portfolio (GitHub Pages)

Static portfolio site built with plain HTML + CSS for `jorgeguzman.github.io`.

## Structure

- `index.html`: Main portfolio page with hero, about, skills, projects, experience, and contact sections.
- `style.css`: All site styling (dark mode, gradients, cards, responsive layout).
- `assets/`: Static assets.
- `assets/jorge-placeholder.svg`: Placeholder profile/hero image.
- `assets/favicon.svg`: Favicon placeholder.
- `projects/`: Project detail pages.
- `projects/project-template.html`: Reusable template for new project pages.

## Edit Content

1. Update personal info in `index.html`:
- Hero name/tagline
- Resume, GitHub, and LinkedIn links
- About text
- Contact email and profile links

2. Update projects in `index.html`:
- Edit the project cards under the `Projects` section.
- Keep each card link pointed to a file in `projects/*.html`.

3. Add a new project page:
- Copy `projects/project-template.html`.
- Rename it (example: `projects/my-new-project.html`).
- Replace placeholder sections with your project details.
- Add the card + link in `index.html`.

4. Replace placeholders:
- Swap `assets/jorge-placeholder.svg` with your own image.
- Replace `assets/favicon.svg` with your favicon if desired.
- Add your real resume PDF and link it from the hero button.

## GitHub Pages (How This Repo Publishes)

For a repository named `jorgeguzman.github.io`, GitHub Pages publishes from the root of the default branch.

Typical flow:

1. Commit your changes.
2. Push to `main`.
3. Wait ~1-3 minutes for deployment.
4. Open `https://jorgeguzman.github.io/`.

If needed, verify in GitHub:
- `Settings` -> `Pages`
- Source should be set to `Deploy from a branch`
- Branch should be `main` and folder `/ (root)`

## Local Preview

Open `index.html` directly in a browser, or run a simple local server from the repo root:

```bash
python3 -m http.server 8080
```

Then visit `http://localhost:8080`.
