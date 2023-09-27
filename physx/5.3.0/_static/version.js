// helper to grab a url and convert it to a js object
async function _fetchJson(url)
{
    console.log(`fetching: ${url}`)
    let response = await fetch(url)
    if (!response.ok)
    {
        throw Error(`unable to retrieve ${url}: ${response.status}`)
    }

    return response.json()
}

// if newer documentation has been published, this function will add an element to the DOM warning the user.
function _populateVersionWarning(project, latest, versions, root)
{
    console.log(`warning check: ${project.version} ${latest}`)

    const versionMismatch = (latest && (project.version != latest));
    if (!versionMismatch)
    {
        return
    }

    // find the message we should display for this version of the docs.  if a custom
    // message wasn't provided, use a default message.
    let message = `This documentation is not the latest for ${project.name}. `
    for (let i = 0; i < versions.length; i++)
    {
        if (versions[i].version == project.version)
        {
            if ("message" in versions[i])
            {
                message = versions[i].message
            }
            break;
        }
    }

    // since the current version is not the latest version, update the message with a link to the latest docs.
    message = `${message} <a href='${root}../${latest}/index.html'>${project.name} ${latest}</a> is the latest version.`
    console.log(message)

    // try to find the main content div
    const elems = document.querySelectorAll("div.wy-nav-content")
    console.log(elems)
    if (elems.length < 1)
    {
        throw Error("could not find main content")
    }

    const content = elems[0]

    // we're trying to inject the following HTML:
    //
    // <div class="omni-version-warning">
    //  <p class="omni-version-warning-content">
    //   This documentation is for an older version of repo_docs.
    //   <a href="../0.9.1/index.html">repo_docs 0.9.1</a> is the latest version.
    //  </p>
    // </div>
    const outer = document.createElement("div")
    outer.classList.add("omni-version-warning")

    const inner = document.createElement("p")
    inner.classList.add("omni-version-warning-content")
    inner.innerHTML = message

    outer.appendChild(inner)

    content.insertBefore(outer, content.firstChild)
}

// this method:
//
// - injects a document version selection box
// - injects a warning message if the docs are not the latest docs
async function _populateVersions()
{
    try
    {
        // sphinx happily puts a relative path to the project root in each of its html files :)
        const root = document.getElementById("documentation_options").getAttribute('data-url_root')
        if (!root)
        {
            throw new Error("unable to find data-url_root in header")
        }

        // grab project information
        const nocache = `?nocache=${(new Date()).getTime()}`
        const project = await _fetchJson(`${root}project.json`)
        let versions = await _fetchJson(`${root}../versions.json${nocache}`)
        const latest = versions.latest
        console.log("latest:", latest)
        versions = versions.versions
        console.log(versions)

        if (versions.length < 1)
        {
            throw new Error("versions.json does not contain any versions")
        }

        // find all of the elements that look like the sidebar (there should just be 1)
        let elems = document.querySelectorAll("div.wy-side-scroll")
        if (elems.length > 0)
        {
            if (elems.length > 1)
            {
                console.log("warning: found more than one side navigation bars")
            }

            // here we're creating the following html:
            //
            // <div class="omni-version">
            //  <span class="omni-version-content">
            //   repo_docs
            //   <select class="omni-version-select">
            //    <option value="../0.9.1/index.html">0.9.1</option>
            //    <option value="../0.8.1/index.html">0.8.1</option>
            //    <option value="../0.7.1/index.html">0.7.1</option>
            //   </select>
            //  </span>
            // </div>
            let sidebar = elems[0]
            let div = document.createElement("div")
            div.classList.add("omni-version")

            let content = document.createElement("span")
            content.classList.add("omni-version-content")
            content.innerHTML = `${project.name} `

            let selector = document.createElement("select")
            selector.classList.add("omni-version-select")

            function _addOption(v)
            {
                console.log(`adding ${v}`)
                let opt = document.createElement("option")
                opt.value = `../${v}/index.html`
                opt.innerHTML = v

                if (v == project.version)
                {
                    console.log(`found self: ${v}`)
                    opt.selected = true
                    foundThisVersion = true
                }

                selector.appendChild(opt)
            }

            let foundThisVersion = false
            for (let i = 0; i < versions.length; i++)
            {
                _addOption(versions[i].version)
            }

            if (!foundThisVersion)
            {
                _addOption(project.version)
            }

            content.appendChild(selector)

            // if the selection changes, change the url
            selector.addEventListener("input", (event) => {
                console.log("selected:", event.target.value)

                let url = root + event.target.value
                console.log("url:", url)
                window.location.assign(url)
            })

            div.appendChild(content)
            sidebar.appendChild(div)

            // inject a warning about out-of-date documentation (if needed)
            _populateVersionWarning(project, latest, versions, root)
        }
        else
        {
            throw Error("unable to find navigation bar to inject version selector")
        }
    }
    catch (e)
    {
        console.log("warning: failed to populate version information:", e)
    }
}

// wait until the page is loaded to modify the DOM
window.addEventListener("load", _populateVersions)
