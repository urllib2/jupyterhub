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
      
      
      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4750, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5500, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      

      

      Symbol.bindElementAction(compId, symbolName, "${_1}", "click", function(sym, e) {
         // insert code for mouse click here
         sym.$("Text3").html("Condition to check");
         sym.$("cursorclick").animate({
             top: '451px',
             left: '591px',
           },800);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_2}", "click", function(sym, e) {
         // insert code for mouse click here
         sym.$("Text3").html("If the condition is true for example the sensor perceives an object at a distance less than 20 cm. Brick display shows an image for 5 seconds");
         sym.$("cursorclick").animate({
             top: '667px',
             left: '583px',
         
           },800);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${_3}", "click", function(sym, e) {
         // insert code for mouse click here
         // insert code for mouse click here
         sym.$("Text3").html("If the condition is false, for example the sensor does not perceive an object at a distance less than 20 cm. The brick light flashes green for 3 seconds");
         sym.$("cursorclick").animate({
             top: '735px',
             left: '283px',
         
           },800);

      });
      //Edge binding end

   })("stage");
   //Edge symbol end:'stage'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

   //=========================================================
   
   //Edge symbol: 'TITRE_1'
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

   })("TITRE_1");
   //Edge symbol end:'TITRE_1'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");