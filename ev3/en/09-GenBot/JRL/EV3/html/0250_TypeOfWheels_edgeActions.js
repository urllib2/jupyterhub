/***********************
* Adobe Edge Animate Composition Actions
*
* Edit this file with caution, being careful to preserve 
* function signatures and comments starting with 'Edge' to maintain the 
* ability to interact with these actions from within Adobe Edge Animate
*
***********************/
(function($, Edge, compId){
var Composition = Edge.Composition, Symbol = Edge.Symbol; // aliases for commonly used Edge classes

   //Edge symbol: 'stage'
   (function(symbolName) {
      
      
      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "document", "compositionReady", function(sym, e) {
         sym.$('car1').hide();
         sym.$('medical1').hide();
         sym.$('tractor1').hide();
         sym.$('research1').hide();
         sym.$('domestic1').hide();
         sym.$('arm1').hide();
         
         // Hide an element 
         sym.$("roueBasique").hide();
         
         // Hide an element 
         sym.$("rouePetite").hide();
         
         // Hide an element 
         sym.$("roueCompetition").hide();
         
         // Hide an element 
         sym.$("petiteg").hide();
         
         // Hide an element 
         sym.$("basicg").hide();
         
         // Hide an element 
         sym.$("motog").hide();
         
         // Hide an element 
         sym.$("grandeg").hide();
         
         // Hide an element 
         sym.$("vpotureg").hide();
         
         
         // Hide an element 
         sym.$("dimensions").hide();
         
         // Hide an element 
         sym.$("roueGeante2").hide();
         
         // Hide an element 
         sym.$("roueGrande").hide();
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${car}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("car").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("car1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("car").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("car1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car1}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").fadeIn(500);
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").fadeIn(500);
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").hide();
         sym.$("motog").hide();
         
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         
         // Hide an element 
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${car}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").fadeIn(500);
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").fadeIn(500);
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").hide();
         sym.$("motog").hide();
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("arm").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("arm1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("arm").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("arm1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm}", "click", function(sym, e) {
         sym.$("rouePetite").fadeIn(500);
         sym.$("roueBasique").hide();
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").fadeIn(500);
         sym.$("basicg").hide();
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").hide();
         sym.$("motog").hide();
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         
         // Hide an element 
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("arm").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("arm1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${arm1}", "click", function(sym, e) {
         sym.$("rouePetite").fadeIn(500);
         sym.$("roueBasique").hide();
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").fadeIn(500);
         sym.$("basicg").hide();
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").hide();
         sym.$("motog").hide();
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("domestic").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("domestic1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("domestic").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("domestic1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").hide();
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").fadeIn(500);
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").hide();
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").hide();
         sym.$("motog").fadeIn(500);
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("domestic").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("domestic1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("domestic").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("domestic1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${domestic1}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").hide();
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").fadeIn(500);
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").hide();
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").hide();
         sym.$("motog").fadeIn(500);
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("tractor").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("tractor1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         sym.$("tractor").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("tractor1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").hide();
         sym.$("roueCompetition").fadeIn(500);
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").hide();
         sym.$("vpotureg").fadeIn(500);
         sym.$("grandeg").hide();
         sym.$("motog").hide();
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("car").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("car1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("car").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("car1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${tractor1}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").hide();
         sym.$("roueCompetition").fadeIn(500);
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").hide();
         sym.$("vpotureg").fadeIn(500);
         sym.$("grandeg").hide();
         sym.$("motog").hide();
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         sym.$("research").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("research1").show();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("research").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("research1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").fadeIn(500);
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").hide();
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").hide();
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").fadeIn(500);
         sym.$("motog").hide();
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research1}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("research").css({'opacity':'0'});
         
         // Hide an element 
         sym.$("research1").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research1}", "mouseleave", function(sym, e) {
         // insert code to be run when the mouse leaves an element
         // Show an element 
         sym.$("research").css({'opacity':'100'});
         
         // Hide an element 
         sym.$("research1").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${research1}", "click", function(sym, e) {
         sym.$("rouePetite").hide();
         sym.$("roueBasique").hide();
         sym.$("roueCompetition").hide();
         sym.$("roueGeante2").fadeIn(500);
         sym.$("roueGrande").hide();
         
         
         // Show an element 
         sym.$("petiteg").hide();
         sym.$("basicg").hide();
         sym.$("vpotureg").hide(); 
         sym.$("grandeg").fadeIn(500);
         sym.$("motog").hide();
         // Show an element 
         sym.$("dimensions").fadeIn(500);
         sym.$("Lego-wheels-that-include-hubs-with-peg-holes").hide();
         sym.$("head").hide();

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'TITRE'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         // insérer le code ici
      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5250, function(sym, e) {
         // insérer le code ic
         sym.stop();

      });
      //Edge binding end

   })("TITRE");
   //Edge symbol end:'TITRE'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");