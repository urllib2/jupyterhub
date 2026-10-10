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
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // insérer le code à exécuter lorsque le chargement de la composition est terminé
         // Masquer un élément 
         
         sym.$("Sol_ex").hide();
         sym.$("pause").hide();
         sym.$("play1").hide();
         sym.$("full_screen2").hide();
         dem=1;
         play=1;
         hint=1;
         demo=1;
         sym.$("full_screen").hide();
         sym.$("long_text").hide();
         sym.getSymbol("video").stop(0);
         
         
         // Masquer un élément 
         sym.$("Solution_Ex1").hide();
         
         
         
         // Masquer un élément 
         
         
         // Masquer un élément 
         sym.$("reflechir").hide();
         
         // Masquer un élément 
         sym.$("Rectangle2").hide();
         sym.$("solu").hide();
         
         // Masquer un élément 
         sym.$("solution").hide();
         
         sym.$("demonstration").hide();
         // Masquer un élément 
         
         
         
         
         sym.play();
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${solu}", "click", function(sym, e) {
         
         
         
         
         // Afficher un élément 
         sym.$("Sol_ex").show();
         sym.$("solu").hide();
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      

            Symbol.bindElementAction(compId, symbolName, "${full_screen}", "click", function(sym, e) {
         
         
         // insérer le code du clic de souris ici
         if (play==1) {
         sym.getSymbol("video").$("demo1")[0].currentTime=0;
         sym.getSymbol("video").$("demo1")[0].pause();
         sym.$('demo').hide();
         sym.$('Hint').hide();
         sym.$('full_screen2').fadeOut(500);
         sym.$('Reflexion').fadeOut(1000);
         sym.$('long_text').fadeOut(1000);
         sym.getSymbol("video").play(0);
           sym.$("full_screen").hide();
         
         setTimeout(function(){ sym.$("full_screen").show();sym.$("full_screen").css({top: 426, left: 506 ,position:'absolute'}); sym.getSymbol("video").$("demo1")[0].play();   }, 1700);
         
          clearTimeout();
         
          play=-1;
         
         
         
         
                 sym.getSymbol("video").$("demo1")[0].ontimeupdate = function() {videoPause()};
                   function videoPause() {
         
                       if ((sym.getSymbol("video").$("demo1")[0].currentTime >=sym.getSymbol("video").$("demo1")[0].duration)&&((x>0)||(y>0))){
         
                           if (!sym.getSymbol("video").isPlaying()){
                             if (play!=1){
                                     sym.$("full_screen").hide();
                                     sym.getSymbol("video").playReverse(1500);
                                     play=1;
                                     sym.getSymbol("video").$("demo1")[0].currentTime=0;
                                     sym.getSymbol("video").$("demo1")[0].pause();
         									sym.$('Reflexion').fadeIn(1500);
         									sym.$('long_text').fadeIn(1500);
                                     setTimeout(function(){  sym.$("demo").show();sym.$("full_screen").show(); sym.$("full_screen").css({top: 456, left: -105}); sym.$("full_screen2").show();}, 1500);
                                     clearTimeout();
         
         
         
                               }
                              else
                                  {
         
                                  }
                                  }
         
         
                           else if (sym.getSymbol("video").isPlayingReverse()){
         
         
         
                             }
                            else if (sym.getSymbol("video").isPlaying()){
                                    sym.$("full_screen").hide();
                                    setTimeout(function(){ sym.getSymbol("video").playReverse(1500);  }, 500);
                                    clearTimeout();
                                    play=1;
                                    sym.getSymbol("video").$("demo1")[0].currentTime=0;
                                   sym.getSymbol("video").$("demo1")[0].pause();
                                   sym.$('Reflexion').fadeIn(1500);
                                   sym.$('long_text').fadeIn(1500);
                                   setTimeout(function(){sym.$("demo").show(); sym.$("full_screen").show(); sym.$("full_screen").css({top: 456, left: -105}); }, 1500);
                                    clearTimeout();
         
         
                                     }
         
         
         
                         }
         
         
                 }
         
         
         }
         else 
         {
         if ((x!=0)||(y!=0)){
         play=1;
         sym.$("full_screen").hide();
         sym.$("full_screen2").fadeIn(1000);
         sym.getSymbol("video").playReverse(1500);
         sym.$('Reflexion').fadeIn(1500);
         sym.$('long_text').fadeIn(1500);
         setTimeout(function(){  sym.$("demo").show(); sym.$("full_screen").show(); sym.$("full_screen").css({top: 456, left: -105}); }, 1500);
          clearTimeout();
         }
         
         
         };
         

      });
         //Edge binding end



      

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 897, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 2000, function(sym, e) {
         // insérer le code ici
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         
         // insérer le code ici
         
         // insérer le code ici// insérer le code ici
         // Afficher un élément 
         sym.$("Compte_rebours").show();
         
         // Afficher un élément 
         sym.$("reflexion").show();
         
          y=3;
         x =0;
         sym.$("secondes").html("0"+x).css("color","#5abf6b");
         sym.$("minutes").html("0"+y).css("color","#5abf6b");
         
         
         var id = setInterval(frame, 1000);
         function frame() {
         if ((x<=1)&&(y==0)){
         
            clearInterval(id);
         	sym.$("full_screen2").hide();
           sym.$("Reflexion").fadeOut(500);
           sym.$("solu").fadeIn(500); 
           sym.$("demonstration").fadeOut(500);
           sym.$("demo").hide();
           sym.$("Hint").hide();
         sym.$("Intro").fadeIn(500);
         
         } 
         
          if (y==0){
          x--;
          if (x<10) {
          sym.$("secondes").html("0"+x).css("color","#5abf6b");
          sym.$("minutes").html("0"+y).css("color","#5abf6b");
          }
          else
          {
           sym.$("secondes").html(x).css("color","#5abf6b");
         sym.$("minutes").html("0"+y).css("color","#5abf6b");
         
          }
          }
          else
          {
          if (x<=10){
         	if (x==0){
         	x=59;
         	y--;
         	sym.$("secondes").html(x).css("color","#5abf6b");
         	sym.$("minutes").html("0"+y).css("color","#5abf6b");
         
         	}
         	else{
         	x--;
         	sym.$("secondes").html("0"+x).css("color","#5abf6b");
         	sym.$("minutes").html("0"+y).css("color","#5abf6b");
              }
          }
         else {
         x--;
         sym.$("minutes").html("0"+y).css("color","#5abf6b");
         sym.$("secondes").html(x).css("color","#5abf6b");
         
         
         }	
         
          }
         
         
         
         
          }
         
         
         sym.stop();

      });
      //Edge binding end

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 0, function(sym, e) {
         // insérer le code ici
         
         // insérer le code ici// insérer le code ici
         // Afficher un élément 
         sym.$("Compte_rebours").show();
         
         // Afficher un élément 
         sym.$("reflexion").show();
         
         var y=0;
         var x = 10;
         sym.$("seconde").html(x).css("color","#5abf6b");
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         var x = 10;
         
         var id = setInterval(frame, 1000);
         function frame() {
         if ((x==0)&&(y==0)){
           clearInterval(id);
           sym.$("Reflexion").hide();
           sym.$("solu").show(); sym.$("demonstration").hide();
           sym.$("demo").hide();
           sym.$("Hint").hide();
         } 
         
          if (y==0){
          x--;
          if (x<10) {
          sym.$("seconde").html("0"+x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
          }
          else
          {
           sym.$("seconde").html(x).css("color","#5abf6b");
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         
          }
          }
          else
          {
          if (x<=10){
          if (x==0){
          x=59;
          y--;
          sym.$("seconde").html(x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         
          }
          else{
          x--;
          sym.$("seconde").html("0"+x).css("color","#5abf6b");
          sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
              }
          }
         else {
         x--;
         sym.$("minuhtes").html("0"+y+":").css("color","#5abf6b");
         sym.$("seconde").html(x).css("color","#5abf6b");
         
         
         }  
         
          }
         
         
         
         
          }
         
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "click", function(sym, e) {
         
         if (dem==1) {
         dem=-1;
         if (hint==-1){
         sym.playReverse(2000);
         }
         sym.$("long_text").show();
         sym.getSymbol("long_text").play();
         sym.$("GREY").show();
         sym.$("demo").animate({top: 450, left: -185 ,position:'absolute'},950);
         }
         sym.$("Intro").hide();
         sym.$("Text3").hide();
         sym.$("pause").show();
         sym.$("demonstration").fadeIn(900);
         sym.$("full_screen").show();
              setTimeout(function(){ sym.$("full_screen2").show(); }, 1000);
                                     clearTimeout();
         sym.$("Hint").fadeOut(500);
         if (hint==-1){
         hint=1;
         sym.playReverse(2000);
         };
         
          sym.getSymbol("video").$("demo1")[0].ontimeupdate = function() {videoPause()};
         function videoPause() {
         	if ((sym.getSymbol("video").$("demo1")[0].currentTime >=sym.getSymbol("video").$("demo1")[0].duration)&&(dem==-1)&&((x!=0)||(y!=0))){
         	sym.$("play1").show();
         	 sym.$("pause").hide();
         	}
         
         }
         
         
         if (sym.getSymbol("video").$("demo1")[0].paused) {
          sym.getSymbol("video").$("demo1")[0].play();
          sym.$("pause").show();
           sym.$("play1").hide();
         }
         else{
         sym.getSymbol("video").$("demo1")[0].pause();
         sym.$("play1").show();
          sym.$("pause").hide();
          }
         
         

      });
      //Edge binding end

      

      

      

      

      

      

      

      Symbol.bindElementAction(compId, symbolName, "${full_screen2}", "click", function(sym, e) {
         // insérer le code du clic de souris ici
         sym.getSymbol("long_text").playReverse();
         
         // Masquer un élément 
         sym.$("demonstration").hide();
         sym.$("full_screen2").hide();
         sym.$("play1").hide();
         sym.$("pause").hide();
         sym.$("Text3").show();
         dem=1;
         // Afficher un élément 
         sym.getSymbol("video").$("demo1")[0].currentTime=0;
         sym.getSymbol("video").$("demo1")[0].pause();
          setTimeout(function(){ sym.$("Hint").show(); }, 1000);
                                     clearTimeout();
         sym.$("demo").animate({top: 563, left: -185 ,position:'absolute'},900);

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${play1}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${play1}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${pause}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${pause}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${solu}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY2").hide();
         sym.$("ORANGE2").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${solu}", "mouseleave", function(sym, e) {
         sym.$("GREY2").show();
         sym.$("ORANGE2").hide();
         

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseover", function(sym, e) {
         // insérer le code à exécuter lorsque vous survolez l'objet à l'aide de la souris
         sym.$("GREY").hide();
         sym.$("ORANGE").show();

      });
      //Edge binding end

      Symbol.bindElementAction(compId, symbolName, "${demo}", "mouseleave", function(sym, e) {
         // Insérer le code à exécuter lorsque la souris quitte un élément
         sym.$("GREY").show();
         sym.$("ORANGE").hide();

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

      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 5000, function(sym, e) {
         // insérer le code ic
         sym.stop();

      });
      //Edge binding end

   })("TITRE");
   //Edge symbol end:'TITRE'

   //=========================================================
   
   //Edge symbol: 'video'
   (function(symbolName) {   
   
      Symbol.bindTriggerAction(compId, symbolName, "Default Timeline", 1500, function(sym, e) {
         // insérer le code ici
         sym.stop()

      });
      //Edge binding end

   })("video");
   //Edge symbol end:'video'

   //=========================================================
   
   //Edge symbol: 'Preloader'
   (function(symbolName) {   
   
   })("Preloader");
   //Edge symbol end:'Preloader'

   //=========================================================
   
   //Edge symbol: 'long_text'
   (function(symbolName) {   
   
   })("long_text");
   //Edge symbol end:'long_text'

})(window.jQuery || AdobeEdge.$, AdobeEdge, "EDGE-6689876");