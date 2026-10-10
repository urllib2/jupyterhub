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
         
         // Hide an element 
         sym.$("mm_ambiante").hide();
         
         // Hide an element 
         sym.$("mm_reflechit").hide();
         
         // Hide an element 
         sym.$("Text_ambiante").hide();
         
         // Hide an element 
         sym.$("Text3_reflechit").hide();
         
         // Hide an element 
         sym.$("Text2_couleur").hide();
         
         // Hide an element 
         sym.$("CASE_DEFINITION_GREEN12").hide();
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${Rect_reflchit_}", "click", function(sym, e) {
         // insert code for mouse click here
         
      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rect_reflchit_}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Show an element 
         sym.$("mm_reflechit").show();
         
         // Show an element 
         sym.$("Text3_reflechit").show();
         sym.$("CASE_DEFINITION_GREEN12").show();
         
         
         // Hide an element 
         sym.$("mm_couleur").hide();
         
         // Hide an element 
         sym.$("Text2_couleur").hide();
         
         // Hide an element 
         sym.$("mm_ambiante").hide();
         
         // Hide an element 
         sym.$("Text_ambiante").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rect_ambiante}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         // Show an element 
         sym.$("mm_ambiante").show();
         
         // Show an element 
         sym.$("Text_ambiante").show();
         
         // Hide an element 
         sym.$("Text3_reflechit").hide();
         
         // Hide an element 
         sym.$("Text2_couleur").hide();
         // Hide an element 
         sym.$("mm_reflechit").hide();
         
         // Hide an element 
         sym.$("mm_couleur").hide();
         sym.$("CASE_DEFINITION_GREEN12").show();
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${Rect_couleur}", "mouseover", function(sym, e) {
         // insert code to be run when the mouse hovers over the object
         
         // Hide an element 
         sym.$("Text_ambiante").hide();
         
         // Hide an element 
         sym.$("Text3_reflechit").hide();
         
         // Hide an element 
         sym.$("mm_reflechit").hide();
         
         // Hide an element 
         sym.$("mm_ambiante").hide();
         // Show an element 
         sym.$("Text2_couleur").show();
         
         // Show an element 
         sym.$("mm_couleur").show();
         
         // Show an element 
         sym.$("CASE_DEFINITION_GREEN12").show();
         

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2000, function(sym, e) {
         sym.stop();

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
   
   //Edge symbol: 'mouse'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 4250, function(sym, e) {
         sym.stop();

      });
      //Edge binding end

   })("mouse");
   //Edge symbol end:'mouse'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");