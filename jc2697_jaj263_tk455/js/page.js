(function() {
  // Save references to DOM elements
  var mainContent = document.getElementById("main-content");
  var menuContainer = document.getElementById("menu-container");
  var menu = document.getElementById("menu");

  // Get the padding above the menu
  var mainContentTop = mainContent.getBoundingClientRect().top + window.scrollY;
  var menuContainerTop = menuContainer.getBoundingClientRect().top + window.scrollY;
  var paddingTop = menuContainerTop - mainContentTop;
  console.log(paddingTop);

  // Move the menu to "stick" on the left side whenever the page is scrolled
  window.addEventListener("scroll", function(e) {
    if (window.scrollY > mainContent.offsetTop - paddingTop) {
      menu.style.top = (window.scrollY - mainContent.offsetTop + paddingTop) + "px";
    }
  }, false);

  // Get the scroll positions of all the checkpoints
  var checkpointEls = document.querySelectorAll(".checkpoint");
  var checkpoints = [];
  for (var i = 0; i < checkpointEls.length; i++) {
    checkpoints.push({
      id: checkpointEls[i].id,
      top: checkpointEls[i].getBoundingClientRect().top + window.scrollY
    });
  }
  checkpoints.sort(function(a, b) {
    if (a.top < b.top) return -1;
    if (a.top > b.top) return 1;
    return 0;
  });

  // Highlight the correct menu element whenever a new checkpoint is passed
  var currHighlighted = checkpoints[0].id;
  function updateHighlight(e) {
    // Get the currently scrolled to element
    var i = 0;
    while (i < checkpoints.length && checkpoints[i].top < window.scrollY + 90) {
      i++;
    }
    if (i > 0) i--;

    // Unhighlight the current element
    document.getElementById("link_" + currHighlighted).classList.remove("is-active");

    // Highlight the new element
    currHighlighted = checkpoints[i].id;
    document.getElementById("link_" + currHighlighted).classList.add("is-active");
  }
  window.addEventListener("scroll", updateHighlight, false);
})();
