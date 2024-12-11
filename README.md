# Repository Summary
**This repository contains code for 9430's 2024 Offseason test bot.** 

# Resources
Project based on [MAXSwerve Java Template v2023.1](https://github.com/REVrobotics/MAXSwerve-Java-Template/releases/tag/v2023.1), using plenty of code from our 2024 competition bot code from a C++ code base.
- [2024 Competition Bot "HPWaffleJet1024"](https://github.com/Cosmonautics/9430-hpwafflejet1024)
- [Java Version of "HPWaffleJet1024"](https://github.com/Cosmonautics/9430-hpwafflejet1024J)
- [Off Season General Repo](https://github.com/bbontrager89/off-season) (Contains documentation and setup guides)
- [Learn Git Commands/Branching here](https://learngitbranching.js.org/)

# Style Guide
This repository follows the conventions and best practices defined below. Please refer to these style guides regularly when writing code or performing code reviews. 

## Java Conventions
We follow [Google Java Style Guide](https://google.github.io/styleguide/javaguide.html) as much as possible. (Formatter file TBD)

## Git Best Practices
**[Write good commit messages](https://cbea.ms/git-commit/)**
- Small, focused/effort based commits.
- Concise summary of changes less than 50 characters in the first line; add description for complex commits less than 72 characters per line.
- Imperative commit messages (**"do code"** not *"doing code"*).
- Capitalize commit messages like a normal sentence. 

**General Repository Conventions**

- Pull Requests are referenced as PR #(ID) and Issues are referenced as simply #(ID) where ID = the ID number of the Issue or Pull Request. 
- Prefer merging over rebasing. If merging to a working code/staging branch, rebasing is preferable. Rebasing should only be done if the source branch being rebased onto a base branch is no longer going to be worked on. 
- [Feature-Branch Workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow)
  - Branch frequently.
  - Use Pull Requests to merge into working feature branches.
  - Commit work to feature branch often, then push branch to remote at the end of the day to keep developers in the loop. 
  - Build code before you commit to avoid committing broken code. Untested code should never be committed to working feature branches.
    - Working feature branches are defined as feature branches that represent working features.
    - Issues may exist with otherwise working features, but that doesn't mean those aren't "working feature branches".
  - Each developer may branch off remote feature branches to commit work into, and rebase those branches onto the remote feature branch via a pull request. 