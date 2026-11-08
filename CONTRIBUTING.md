# Contributing Guidelines

To contribute to the FRC 9584 robot codebase, please follow the rules below:

## Branching Guidelines

### General Rules

- **Never commit directly to `main`.**
  - `main` should always contain stable, deployable code.
  - Direct commits can introduce bugs and break the build.

- **Create a new branch from `main` for every change.**
  - Keeps work isolated and avoids conflicts.
  - Makes code review and testing easier.
  - Maintains a clean and organized commit history.

- **Branches should be short-lived (trunk-based development)**
  - Each branch should be focused on a small, incremental improvement.
  - Ideally most changes should have the ability to be merged into main within a day.
  - Avoid long-lived feature branches, this is to prevent "merge hell" and keep the purpose of the branch within scope.

### Branch Naming Convention

Use the following format when naming your branches:
`github-username/<issue-number-if-applicable>-<short-description>`

**Example:**

```bash
git checkout -b bob/[issue-number'if there is one']-fix-drivetrain-code
```

This naming convention helps:

- Keep branches easy to search and understand.
- Identify who is responsible for each change.
- Avoid naming collisions when multiple contributors are working on similar tasks.

## Commits

- Keep commits small and focused.
- Commit message format:

  ```
  Short summary (max 50 chars)

  Longer description if needed (wrap at 72 chars).
  ```

- Start messages with a verb: "Add", "Fix", "Update", etc. (This is standard)
- Keep commit message relevant to the issue, feature, etc. being addressed.

**Example:**

```bash
git add .
git commit -m "Fix Drivetrain Code"
```

## Pushing to GitHub

After committing your changes, push your branch to GitHub using:

```bash
git push origin HEAD
```

This command:
- Pushes your current branch to the remote repository
- Uses `HEAD` to automatically reference the current branch you're working on
- Creates the branch on GitHub if it doesn't already exist
- Is equivalent to `git push origin <branch-name>` but more convenient

## Pull Requests

- Always open a pull request (PR) into `main`.
- PR title format: `Type: short description` (e.g., `Feature: add login endpoint`).
- PR description must explain:
  - What the change is
  - Why it's needed
  - How it was implemented
- Link any related issues, this means **include associated Issue ID** with every PR.
  - This ensures that each change is linked to an issue, providing context for the change and ensuring thorough documentation of the change history in the Issue tracker.

## Merging

- Do not merge your own PR without review.
- At least two approvals are required.
- Those reviewing should demonstrate they are comfortable with the codebase, as well as are responsible reviewers.
- Resolve all merge conflicts before merging.
- Prefer "Squash and merge" to keep a clean history.

## Automated and Unit Testing

- During the off-season, unit testing is **required** for any changes relating to the robot code.
  - This ensures that we maintain a high level of code quality and reliability even when not actively competing.
- Tests should be written **before** or **concurrently with the code changes**.
- Every PR should have code that is tested thoroughly.
  - Ensure that the changes in your PR have adequate test coverage. If tests are missing or incomplete, the PR will not be merged until the tests are added or completed.
  - **Run all tests** locally and confirm that the PR passes all tests before submitting.

## Variable Naming

- All code variables should be formatted using the [Google style guide for java](https://google.github.io/styleguide/javaguide.html#s5-naming).
  - Constants should be `ALL_CAPS`
  - Member variables are `camelCase` with no prefixes/sufixes.
  - Class names are `PascalCase`
  - Assume other variables are `camelCase`

## Formatting

- All code in a PR should be formatted, this can be done by running:

```bash
./gradlew spotlessApply
```

If your code isn't formatted your build will fail.

To add this to autosave in VSCode do the following:

First install `google-java-format`:

**For Mac:**
```bash
brew install google-java-format
```

**For Linux and Windows**, download for your architecture on the latest release in the [Releases tab](https://github.com/google/google-java-format/releases). Remember where you install it.

Then install the extension `emeraldwalk.RunOnSave` by running:

```bash
code --install-extension emeraldwalk.RunOnSave
```

Or by searching `emeraldwalk` in `VSCode: Extensions` and installing `Run on Save`.

Then add this to your `settings.json` in VSCode (Opened by pressing ctrl+shift+p and typing "open user settings"):

For Mac:
```json
"emeraldwalk.runonsave": {
  "commands": [
    {
      "match": ".*\\.java$",
      "cmd": "google-java-format -r ${file}"
    }
  ]
}
```

For Linux:
```json
"emeraldwalk.runonsave": {
  "commands": [
    {
      "match": ".*\\.java$",
      // Assuming you downloaded to ~/.local/bin/google-java-format and it's in the PATH
      "cmd": "google-java-format -r \"${file}\""
      // If you don't want it in the PATH just use the full path to the binary here
    }
  ]
}
```

For Windows:
```json
"emeraldwalk.runonsave": {
  "commands": [
    {
      "match": ".*\\.java$",
      // Assuming you downloaded to C:Users\My Name\Downloads
      "cmd": "\"C:\\Users\\My Name\\Downloads\\google-java-format_windows-x86-64.exe\" -r \"${file}\""
    }
  ]
}
```

### Note:
Formatting on save has a bit of delay

## General

- Keep `main` always deployable.
- Use and respect `.gitignore`.
- Document anything non-obvious in code or README.

## Using WPILib + Cursor IDE

- Cursor IDE is an IDE with built in GenAI features, which allow you to auto generate code, which can improve efficiency and quality of your code.
- Unfortunately, both Cursor and WPILib come packaged with VSCode. So combining them is not straightforward.
- Instructions below will help you pull WPILib modules into Cursor so you can use both together.

1. Download and install WPILib: https://github.com/wpilibsuite/allwpilib/releases
2. Find the WPILib VS Code extensions directory, e.g. ~/wpilib/YEAR/vsCodeExtensions
3. Download and install Cursor: https://www.cursor.com/
4. In Cursor:

    a. Open the Command Palette (Ctrl+Shift+P)

    b. Select "Extensions: Install from VSIX"

    c. Navigate to the extensions directory from step 2 above and select all .vsix files there

5. Restart Cursor (May not be necessary, idk)

## Installing Phoenix6 Extensions into VSCode

Follow instructions at https://v6.docs.ctr-electronics.com/en/stable/docs/installation/installation-frc.html
