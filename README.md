# mushr-website

The MuSHR website is generated using the Hugo static site generator. Please refer to Hugo's documentation for any major changes. You can host the site locally by running `hugo server -D` in this directory. Do reach out to @johannesburg or @schmittle before pushing major changes.

## Pushing website edits
Pushing changes is very simple. Clone this repository, make your changes, then simply execute `./deploy.sh "<commit message>"` with a deployment commit message. You can run `chmod +x deploy.sh` to make the deploy script executable. That's it! You're ready to push to the MuSHR website.

## Editing content
All website content can be found in the `content` folder. Merely update one of the existing markdown files, or add your own (I recommend merely duplicating an existing one, unless you're making extensive changes).
