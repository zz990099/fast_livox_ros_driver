Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).

## Hardware Testing Contributions

We welcome PRs that add new hardware test results to the README table.

**To contribute a hardware test result:**

1. Open a hardware testing report issue first (use the *Hardware Testing Report* template) and record your findings there.
2. Once verified, open a PR that updates the CPU-usage table in `README.md` with your platform and measured values.
3. Include a reference to the corresponding issue in your PR description.

**PR review checklist (maintainers):**

- [ ] Platform details (CPU model, architecture, OS, ROS 2 distro) are clearly stated.
- [ ] CPU usage numbers are plausible and the measurement method is described.
- [ ] The README table row follows the existing format.
- [ ] The PR references a corresponding issue with raw test data.
