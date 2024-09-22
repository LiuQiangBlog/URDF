# How to contribute
If you are interested in contributing, please give this guide a quick read to make sure the code you write is formatted correctly. 

# Formatting and style
The formatting is done with clang-format. Running tests will check the formatting, and if reformatting is needed you can do so by running `ament_clang_format --config <path_to_clang_config> --reformat.

# Error handling
- Code must compile with all warnings turned on
- Exceptions are ok. Use the RdlException though and provide a useful message

# Documentation
Any methods, public or private, must be commented with doxygen style comments as well as any member variables created. There are lots of comments in the code, it should be pretty easy to just pattern match something that's already there.

# Bug fixes
If you believe you have found a bug and would like to provide a fix(thanks!), you are required to also submit in your pull request a unit test that exposes the bug you are fixing. If you aren't interested in fixing it yourself, create an issue giving as much information as possible as well as a failure case so the developer(s) can reproduce the bug. Also make sure that your pull request is back onto the develop branch.

# New features
If you are interested in adding a completely new feature, please follow these steps

1. Create an issue so folks know you're interested in the feature and so others can weigh in. This is important because some features the developer(s) may not want in RDL, so it's encouraged that you get the thumbs up to keep you from wasting time. Also others might be able to help or have good implementation ideas.
2. Fork RDL. I think you'll have to put your fork on GitLab for the PR back to upstream to work properly.
3. Create your branch off the `develop` branch. If the issue created for this feature is #123, then please make your feature branch named `feature/RDL-123`.
4. Implement the feature in your fork, unit testing as rigorously as possible all added functionality
5. Once your feature is implemented, tested, and documented and it's ready for prime time, create a merge request to the develop branch and the developer(s) can start reviewing it.
6. Once it has gotten the thumbs up from reviewers and the build has passed in the CI pipeline, it'll get merged in!
