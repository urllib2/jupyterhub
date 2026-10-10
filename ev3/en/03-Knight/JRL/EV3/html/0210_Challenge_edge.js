/*jslint */
/*global AdobeEdge: false, window: false, document: false, console:false, alert: false */
(function (compId) {

    "use strict";
    var im='images/',
        aud='media/',
        vid='media/',
        js='js/',
        fonts = {
            'Digital-7': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            '\'Digital tech\'': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />',
            'MagistralC': '<link rel=\"stylesheet\" href=\"fonts.css\" type=\"text/css\" media=\"screen\" title=\"\" charset=\"utf-8\" />'        },
        opts = {
            'gAudioPreloadPreference': 'auto',
            'gVideoPreloadPreference': 'auto'
        },
        resources = [
        ],
        scripts = [
            js+"jquery-2.0.3.min.js",
            js+"jquery.ui.touch-punch.min.js",
            js+"jquery-ui.min.js"
        ],
        symbols = {
            "stage": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "both",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['85px', '5px', '1440px', '1080px', 'auto', 'auto'],
                            clip: 'rect(0px 1440px 1080px 0px)',
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px'],
                            transform: [[],[],[],['1.00694']]
                        },
                        {
                            id: 'long_text',
                            symbolName: 'long_text',
                            type: 'rect',
                            rect: ['181', '99', '1085', '469', 'auto', 'auto']
                        },
                        {
                            id: 'Intro',
                            type: 'group',
                            rect: ['241px', '101px', '1019', '177', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['81px', '15px', '962px', '436px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_BLUE.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['182px', '-4px', '816px', '157px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> </span>Program your knight to:<span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">:</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​-</span> Make the sound <span style=\"font-weight: 700;\">\"Forward\"</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> </span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- Wait for 1 second</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- Go forward 5 rotations with </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 700;\">power=40 &nbsp;</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> &nbsp; &nbsp;&nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- Stop for 2 seconds</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> - Make the sound&nbsp;</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 800; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">\"Backwards\"</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- Wait for 1 second</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- </span>Go back to return to the starting point</p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- Make the sound&nbsp;</span><span style=\"font-weight: 800; font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">\"crunching\"</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> </span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p>",
                                align: "left",
                                font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "500", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'ROBOT07',
                                type: 'image',
                                rect: ['-30px', '63px', '160px', '161px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"RobotP.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Reflexion',
                            type: 'group',
                            rect: ['746px', '226px', '843', '745', 'auto', 'auto'],
                            clip: 'rect(0px 758.625px 745px 0px)',
                            c: [
                            {
                                id: 'reflexion',
                                type: 'image',
                                rect: ['149px', '257px', '651px', '475px', 'auto', 'auto'],
                                clip: 'rect(18.181884765625px 651px 475px 36.36376953125px)',
                                fill: ["rgba(0,0,0,0)",im+"reflexion.png",'0px','0px'],
                                transform: [[],[],[],['0.86866','0.86866']]
                            },
                            {
                                id: 'Compte_rebours',
                                type: 'group',
                                rect: ['109px', '-86px', '311px', '296px', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'timer',
                                    type: 'image',
                                    rect: ['87px', '338px', '268px', '249px', 'auto', 'auto'],
                                    clip: 'rect(-10.733642578125px 268px 262.48748779296875px 1.01953125px)',
                                    fill: ["rgba(0,0,0,0)",im+"timerTransp.png",'0px','0px'],
                                    transform: [[],[],[],['0.5603','0.35816']]
                                },
                                {
                                    id: 'secondes',
                                    type: 'text',
                                    rect: ['203px', '441px', '111px', '90px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">00​</p>",
                                    align: "center",
                                    font: ['Digital-7', [52, "px"], "rgba(41,201,121,1.00)", "900", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'minutes',
                                    type: 'text',
                                    rect: ['165px', '441px', '111px', '90px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(90, 191, 107);\">&nbsp; </span><span style=\"font-size: 50px;\">0​</span></p>",
                                    align: "left",
                                    font: ['Digital-7', [52, "px"], "rgba(41,201,121,1.00)", "900", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'point',
                                    type: 'text',
                                    rect: ['194px', '442px', '111px', '90px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(90, 191, 107);\">&nbsp; </span><span style=\"font-size: 50px;\">​</span>:</p>",
                                    align: "left",
                                    font: ['Digital-7', [52, "px"], "rgba(41,201,121,1.00)", "900", "none", "", "break-word", "normal"]
                                }]
                            }]
                        },
                        {
                            id: 'demonstration',
                            type: 'group',
                            rect: ['928px', '397px', '677', '381', 'auto', 'auto'],
                            c: [
                            {
                                id: 'video',
                                symbolName: 'video',
                                type: 'rect',
                                rect: ['-855', '-406px', '1920', '1080', 'auto', 'auto'],
                                clip: 'rect(105.555419921875px 1920px 967.499755859375px 0px)'
                            },
                            {
                                id: 'cache_cadre',
                                type: 'image',
                                rect: ['-848px', '481px', '1450px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                            },
                            {
                                id: 'full_screen',
                                type: 'image',
                                rect: ['-109px', '457px', '48px', '48px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'full_screen2',
                            type: 'image',
                            rect: ['1016px', '172px', '35px', '35px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                        },
                        {
                            id: 'Sol_ex',
                            type: 'image',
                            rect: ['422px', '539px', '792px', '378px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Solution.PNG",'0px','0px']
                        },
                        {
                            id: 'solu',
                            type: 'group',
                            rect: ['-184px', '563px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE2',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREY2',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text32',
                                type: 'text',
                                rect: ['910px', '18px', '123px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Solution</p>",
                                align: "left",
                                font: ['MagistralC', [28, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'demo',
                            type: 'group',
                            rect: ['-185px', '563px', '1051px', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1054.28564453125px 82px 900px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3',
                                type: 'text',
                                rect: ['921px', '14px', '99px', '32px', 'auto', 'auto'],
                                clip: 'rect(5.479248046875px 103.4658203125px 35.095703125px 12.32861328125px)',
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Demo</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'play1',
                                type: 'image',
                                rect: ['941px', '2px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"play1.png",'0px','0px']
                            },
                            {
                                id: 'pause',
                                type: 'image',
                                rect: ['941px', '0px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"pause.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Hint',
                            type: 'group',
                            rect: ['847px', '364px', '656px', '266', 'auto', 'auto'],
                            clip: 'rect(-33.33349609375px 656px 130.705810546875px -262.7451171875px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'Hint_green',
                                type: 'image',
                                rect: ['-280px', '-40px', '656px', '0px', 'auto', 'auto'],
                                clip: 'rect(0px 656px 241.58349609375px 0px)',
                                opacity: '0.3',
                                fill: ["rgba(0,0,0,0)",im+"Hint_green.png",'0px','0px'],
                                transform: [[],[],[],['1','0.60428']]
                            },
                            {
                                id: 'Text',
                                type: 'text',
                                rect: ['-223px', '32px', '550px', '123px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">​The crunshing sound is in the expression category</p>",
                                align: "left",
                                font: ['\'Arial Black\', Gadget, sans-serif', [24, "px"], "rgba(0,0,0,0.00)", "900", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            },
                            {
                                id: 'indices',
                                type: 'image',
                                rect: ['58px', '-32px', '150px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"indices.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1_1',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
                        }
                    ],
                    style: {
                        '${Stage}': {
                            isStage: true,
                            rect: ['null', 'null', '1650px', '1080', 'auto', 'auto'],
                            overflow: 'hidden',
                            fill: ["rgba(240,240,240,0)"]
                        }
                    }
                },
                timeline: {
                    duration: 5250,
                    autoPlay: false,
                    data: [
                        [
                            "eid368",
                            "left",
                            1000,
                            0,
                            "linear",
                            "${Hint_green}",
                            '-280px',
                            '-280px'
                        ],
                        [
                            "eid356",
                            "color",
                            1648,
                            352,
                            "linear",
                            "${Text}",
                            'rgba(0,0,0,0)',
                            'rgba(0,0,0,1.00)'
                        ],
                        [
                            "eid371",
                            "top",
                            1000,
                            1000,
                            "linear",
                            "${Hint_green}",
                            '-10px',
                            '-40px'
                        ],
                        [
                            "eid307",
                            "opacity",
                            1000,
                            1000,
                            "linear",
                            "${Hint_green}",
                            '0.3',
                            '1'
                        ],
                        [
                            "eid449",
                            "width",
                            0,
                            0,
                            "linear",
                            "${full_screen2}",
                            '35px',
                            '35px'
                        ],
                        [
                            "eid450",
                            "height",
                            0,
                            0,
                            "linear",
                            "${full_screen2}",
                            '35px',
                            '35px'
                        ],
                        [
                            "eid284",
                            "height",
                            1000,
                            905,
                            "linear",
                            "${Hint_green}",
                            '0px',
                            '228px'
                        ],
                        [
                            "eid447",
                            "top",
                            0,
                            0,
                            "linear",
                            "${full_screen2}",
                            '172px',
                            '172px'
                        ],
                        [
                            "eid448",
                            "left",
                            0,
                            0,
                            "linear",
                            "${full_screen2}",
                            '1016px',
                            '1016px'
                        ]
                    ]
                }
            },
            "video": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            preload: 'auto',
                            rect: ['-239px', '-15px', '1920', '1080', 'auto', 'auto'],
                            source: ['media/demo2%284%29.webm'],
                            id: 'demo1',
                            type: 'video',
                            clip: 'rect(153px 1651.6951904296875px 931.61279296875px 277.8624267578125px)',
                            tag: 'video'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '1920px', '1080px']
                        }
                    }
                },
                timeline: {
                    duration: 1500,
                    autoPlay: true,
                    data: [
                        [
                            "eid474",
                            "left",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '-549px',
                            '-239px'
                        ],
                        [
                            "eid472",
                            "scaleY",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '0.55',
                            '1'
                        ],
                        [
                            "eid475",
                            "clip",
                            0,
                            692,
                            "linear",
                            "${demo1}",
                            [153,1651.6951904296875,931.61279296875,277.8624267578125],
                            [119.666748046875,1661.545654296875,988.9443359375,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid473",
                            "scaleX",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '0.55',
                            '1'
                        ],
                        [
                            "eid471",
                            "top",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '212px',
                            '-15px'
                        ]
                    ]
                }
            },
            "long_text": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            rect: ['48px', '6px', '1019', '177', 'auto', 'auto'],
                            id: 'IntroCopy',
                            type: 'group',
                            transform: [[], [], [], ['0.7', '0.7']],
                            c: [
                            {
                                type: 'image',
                                id: 'CASE_DEFINITION_BLUECopy',
                                rect: ['75px', '22px', '962px', '441px', 'auto', 'auto'],
                                fill: ['rgba(0,0,0,0)', 'images/CASE_DEFINITION_BLUE.png', '0px', '0px']
                            },
                            {
                                rect: ['182px', '0px', '816px', '157px', 'auto', 'auto'],
                                font: ['Tahoma, Geneva, sans-serif', [34, 'px'], 'rgba(0,0,0,1.00)', '500', 'none', 'normal', 'break-word', 'normal'],
                                textStyle: ['', '', '', '', 'none'],
                                id: 'Text5Copy',
                                text: '<p style=\"margin: 0px;\">​<span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">​</span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">Programmer votre cheval pour :</span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">​- Emettre le son </span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 800; font-style: normal; text-decoration: none; font-size: 32px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">\"Forward\"</span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">- Attendre 1 seconde </span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">- Avancer 5 rotations avec </span><span style=\"font-weight: 800; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">une puissance =40 </span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">&nbsp; &nbsp; &nbsp;&nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">- S\'arrêter 2 secondes</span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\"> - Emettre le son&nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 800; font-style: normal; text-decoration: none; font-size: 32px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">\"Backwards\"</span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">- Attendre 1 seconde </span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">- Reculer pour retourner au point de départ 5&nbsp;</span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">- Emettre le son </span><span style=\"font-weight: 800; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-size: 32px;\">\"crunching\"</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\"> </span></p><p style=\"margin: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">​</span></p>',
                                align: 'left',
                                type: 'text'
                            },
                            {
                                type: 'image',
                                id: 'ROBOT07Copy',
                                rect: ['-48px', '34px', '165px', '168px', 'auto', 'auto'],
                                fill: ['rgba(0,0,0,0)', 'images/RobotP.png', '0px', '0px']
                            }]
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '1085px', '469px']
                        }
                    }
                },
                timeline: {
                    duration: 1000,
                    autoPlay: true,
                    data: [
                        [
                            "eid437",
                            "scaleX",
                            0,
                            1000,
                            "linear",
                            "${IntroCopy}",
                            '1',
                            '0.7'
                        ],
                        [
                            "eid438",
                            "scaleY",
                            0,
                            1000,
                            "linear",
                            "${IntroCopy}",
                            '1',
                            '0.7'
                        ]
                    ]
                }
            },
            "TITRE_1_1": {
                version: "6.0.0",
                minimumCompatibleVersion: "5.0.0",
                build: "6.0.0.400",
                scaleToFit: "none",
                centerStage: "none",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            type: 'text',
                            id: 'Text6Copy2',
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Challenge</p>',
                            rect: ['-56px', '-3px', '655px', '57px', 'auto', 'auto'],
                            align: 'center',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            textStyle: ['', '', '', '', 'none']
                        },
                        {
                            type: 'text',
                            id: 'Text6',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">challenge</span></p>',
                            rect: ['114px', '-4px', '699px', '57px', 'auto', 'auto'],
                            align: 'center',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
                            textStyle: ['', '', '', '', 'none']
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '390px', '57px']
                        }
                    }
                },
                timeline: {
                    duration: 5250,
                    autoPlay: true,
                    data: [
                        [
                            "eid132",
                            "clip",
                            0,
                            500,
                            "linear",
                            "${Text6}",
                            [0,0,57,0],
                            [0,60,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid112",
                            "clip",
                            500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,60,57,0],
                            [0,100,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid114",
                            "clip",
                            750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,100,57,0],
                            [0,140,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid116",
                            "clip",
                            1000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,140,57,0],
                            [0,160,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid118",
                            "clip",
                            1250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,160,57,0],
                            [0,190,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid120",
                            "clip",
                            1500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,190,57,0],
                            [0,220,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid122",
                            "clip",
                            1750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,220,57,0],
                            [0,260,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid124",
                            "clip",
                            2000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,260,57,0],
                            [0,270,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid126",
                            "clip",
                            2250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,270,57,0],
                            [0,290,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid128",
                            "clip",
                            2500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,290,57,0],
                            [0,320,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid130",
                            "clip",
                            2750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,320,57,0],
                            [0,350,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid157",
                            "clip",
                            3000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,350,57,0],
                            [0,380,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid159",
                            "clip",
                            3250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,380,57,0],
                            [0,405,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid161",
                            "clip",
                            3500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,405,57,0],
                            [0,475,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid165",
                            "clip",
                            3750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,475,57,0],
                            [0,505,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,505,57,0],
                            [0,530,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid199",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,0],
                            [0,540,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid201",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,540,57,0],
                            [0,565,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid203",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,565,57,0],
                            [0,595,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid205",
                            "clip",
                            5000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,0],
                            [0,519.83837890625,57,-78.28271484375],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0210_Challenge_edgeActions.js");
})("EDGE-6689876");
