"use strict";StackExchange.mockups=function(){function e(e,t,n,i,r){function a(e,t,n){for(var i=-1,r=-1;;){if(r=t.indexOf(e,r+1),-1==r)break;(0>i||Math.abs(r-n)<Math.abs(r-i))&&(i=r)}return i}return e.replace(new RegExp("<!-- Begin mockup[^>]*? -->\\s*!\\[[^\\]]*\\]\\((http://[^ )]+)[^)]*\\)\\s*<!-- End mockup -->","g"),function(e,s,o){var l={"payload":s.replace(/[^-A-Za-z0-9+&@#\/%?=~_|!:,.;\(\)]/g,""),"pos":a(e,t,o),"len":e.length};return-1===l.pos?e:(r.push(l),e+"\n\n"+n+i+"-"+(r.length-1)+"%")})}function t(){StackExchange.externalEditor.init({"thingName":"mockup","thingFinder":e,"getIframeUrl":function(e){var t="/plugins/mockups/editor";return e&&(t+="?edit="+encodeURIComponent(e)),t},"buttonTooltip":"UI wireframe","buttonImageUrl":"/content/balsamiq/wmd-mockup-button.png?v=4","onShow":function(e){window.addMockupToEditor=e},"onRemove":function(){window.addMockupToEditor=null;try{delete window.addMockupToEditor}catch(e){}}})}return{"init":t}}(),StackExchange.schematics=function(){function e(){if(!window.postMessage)return i;var e=document.createElement("div");e.innerHTML="<svg/>";var t="http://www.w3.org/2000/svg"==(e.firstChild&&e.firstChild.namespaceURI);if(!t)return i;var n=navigator.userAgent;return/Firefox|Chrome/.test(n)?s:/Apple/.test(navigator.vendor)||/Opera/.test(n)?a:r}function t(e,t,n,i,r){function a(e,t,n){for(var i=-1,r=-1;;){if(r=t.indexOf(e,r+1),-1==r)break;(0>i||Math.abs(r-n)<Math.abs(r-i))&&(i=r)}return i}return e.replace(new RegExp("<!-- Begin schematic[^>]*? -->\\s*!\\[[^\\]]*\\]\\((http://[^ )]+)[^)]*\\)\\s*<!-- End schematic -->","g"),function(e,s,o){var l={"payload":s.replace(/[^-A-Za-z0-9+&@#\/%?=~_|!:,.;\(\)]/g,""),"pos":a(e,t,o),"len":e.length};return-1===l.pos?e:(r.push(l),e+"\n\n"+n+i+"-"+(r.length-1)+"%")})}function n(){var n;StackExchange.externalEditor.init({"thingName":"schematic","thingFinder":t,"getIframeUrl":function(e){var t="/plugins/schematics/editor";return e&&(t+="?edit="+encodeURIComponent(e)),t},"buttonTooltip":"Schematic","buttonImageUrl":"/content/electronics/img/wmd-schematic-button.png?v=1","checkSupport":function(){var t=e();switch(t){case s:return!0;case a:return confirm("Your browser is not officially supported by the schematics editor; however it has been reported to work. Launch the editor?");case r:return confirm("Your browser is not officially supported by the schematics editor; it may or may not work. Launch the editor anyway?");case i:return alert("Sorry, your browser does not support all the necessary features for the schematics editor."),!1}},"onShow":function(e){var t=$("<div class='popup' />").css("z-index",1111).text("Loading editor").appendTo("body").show().addSpinner({"marginLeft":5}).center({"dy":-200});$("<div style='text-align:right;margin-top: 10px' />").append($("<button>cancel</button>").click(function(){t.remove(),e()})).appendTo(t),n=function(n){if(n=n.originalEvent,"https://www.circuitlab.com"===n.origin){n.data||e();var i=$.parseJSON(n.data);if(i&&"success"===i.load)return t.remove(),void 0;if(i&&i.edit_url&&i.image_url){i.fkey=StackExchange.options.user.fkey;var r=$("<div class='popup' />").css("z-index",1111).appendTo("body").show(),a=function(){r.text("Storing image").addSpinner({"marginLeft":5}).center(),$.post("/plugins/schematics/save",i).done(function(t){r.remove(),e(t.img)}).fail(function(e){if(409===e.status){var t="Storing aborted";e.responseText.length<200&&(t=e.responseText),r.text(t+", will retry shortly").addSpinner({"marginLeft":5}).center(),setTimeout(a,1e4)}else r.remove(),alert("Failed to upload the schematic image.")})};a()}}},$(window).on("message",n)},"onRemove":function(){$(window).off("message",n)}})}var i=0,r=1,a=2,s=3;return{"init":n}}(),StackExchange.externalEditor=function(){function e(e){function t(e,t){function p(t){function i(){StackExchange.helpers.closePopups(x.add(r)),u()}var r,o=g||v.caret(),l=v[0].value||"",d=t?t.pos:o.start,p=t?t.len:o.end-o.start,h=l.substring(0,d),f=l.substring(d+p);g=null;var m=function(t,r){if(!t)return setTimeout(i,0),v.focus(),void 0;StackExchange.navPrevention.start();var a=void 0===r?n(t):r,s=h.replace(/(?:\r\n|\r|\n){1,2}$/,""),l=s+a+f.replace(/^(?:\r\n|\r|\n){1,2}/,""),c=o.start+a.length-h.length+s.length;setTimeout(function(){e.textOperation(function(){v.val(l).focus().caret(c,c)}),i()},0)},x=null;if(a){var b=a(t?t.payload:null);x=$("<iframe>",{"src":b})}else{var k=s(t?t.payload:null);x=$(k)}x.addClass("esc-remove").css({"position":"fixed","top":"2.5%","left":"2.5%","width":"95%","height":"95%","background":"white","z-index":1001}),$("body").loadPopup({"html":x,"target":$("body"),"lightbox":!0}).done(function(){$(window).resize(),c(m)})}$('<style type="text/css"> .wmd-'+i+"-button span { background-position: 0 0; } .wmd-"+i+"-button:hover span { background-position: 0 -40px; }</style>)").appendTo("head");var h,f,g,m=e.getConverter().hooks,v=$("#wmd-input"+t);m.chain("preConversion",function(e){var t=(e.match(/%/g)||[]).length,n=v[0].value||"";return h=new Array(t+2).join("%"),f=[],r(e,n,h,i,f)}),m.chain("postConversion",function(e){return e.replace(new RegExp(h+i+"-(\\d+)%","g"),function(e,t){return"<sup><a href='#' class='edit-"+i+"' data-id='"+t+"'>edit the above "+i+"</a></sup>"})});var x="The "+i+" editor does not support touch devices.",b=!1;$("#wmd-preview"+t).on("touchend",function(){b=!0}).on("click","a.edit-"+i,function(){return b?(alert(x),b=!1,!1):(b=!1,(!d||d())&&p(f[$(this).attr("data-id")]),!1)}),$("#wmd-input"+t).keyup(function(e){e.shiftKey||e.altKey||e.metaKey||!e.ctrlKey||77!==e.which||(!d||d())&&p()}),setTimeout(function(){var e=($("#wmd-button-bar"+t),$("#wmd-image-button"+t)),n=parseInt(e.css("left"));e.nextAll("li").each(function(){var e=$(this),t=parseInt(e.css("left"));e.css("left",t+25)});var r=$("<li class='wmd-button wmd-"+i+"-button' style='left:"+(n+25)+"px' id='wmd-"+i+"-button"+t+"' title='"+o+" Ctrl-M' />").insertAfter(e),a=!1,s=$("<span />").css({"backgroundImage":"url("+l+")"}).appendTo(r).on("touchend",function(){a=!0}).click(function(){return a?(alert(x),a=!1,void 0):(a=!1,(!d||d())&&p(),void 0)});$.browser.msie&&s.mousedown(function(){g=v.caret()})},0)}function n(e){return('\n\n<!-- Begin {THING}: In order to preserve an editable {THING}, please\n     don\'t edit this section directly.\n     Click the "edit" link below the image in the preview instead. -->\n\n![{THING}]('+e+")\n\n<!-- End {THING} -->\n\n").replace(/{THING}/g,i)}var i=e.thingName,r=e.thingFinder,a=e.getIframeUrl,s=e.getDivContent,o=e.buttonTooltip,l=e.buttonImageUrl,c=e.onShow,u=e.onRemove||function(){},d=e.checkSupport;StackExchange.MarkdownEditor.creationCallbacks.add(t)}return{"init":e}}();