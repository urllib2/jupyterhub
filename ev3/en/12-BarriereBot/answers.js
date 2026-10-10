/*!
 * Answers.js
*/

(function( root, factory ) {
	if( typeof define === 'function' && define.amd ) {
		// AMD. Register as an anonymous module.
		define( function() {
			root.Answers = factory();
			return root.Answers;
		} );
	} else if( typeof exports === 'object' ) {
		// Node. Does not work with strict CommonJS.
		module.exports = factory();
	} else {
		// Browser globals.
		root.Answers = factory();
	}
}( this, function() {

	'use strict';

	var Answers;


	var AnswersQ1matrix =
[[1,0,0,0],
 [1,0,0,1],
 [0,1,1,0],
 [1,0,0,1],
 [0,0,0,1],
 [0,1,0,0],
 [0,0,1,0],
 [0,0,0,1],
 [1,1,1,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0]];
	var AnswersQ2matrix =
[[1,0,0,0],
 [0,0,1,0],
 [0,1,0,0],
 [1,0,0,1],
 [0,1,0,1],
 [0,0,1,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0],
 [0,0,0,0]];
	

	// --------------------------------------------------------------------//
	// ------------------------------- API --------------------------------//
	// --------------------------------------------------------------------//


	Answers = {
		
		getAnswersQ1matrix: function() {
			return ( AnswersQ1matrix);
		},
		getAnswersQ2matrix: function() {
			return ( AnswersQ2matrix );
		},
		
	};

	return Answers;

}));
