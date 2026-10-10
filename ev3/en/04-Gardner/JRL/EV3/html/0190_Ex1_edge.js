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
                            id: 'TITRE',
                            symbolName: 'TITRE_1_2',
                            type: 'rect',
                            rect: ['396px', '28px', '390', '57', 'auto', 'auto']
                        },
                        {
                            id: 'long_text',
                            symbolName: 'long_text',
                            type: 'rect',
                            rect: ['221px', '88px', '1085', '469', 'auto', 'auto']
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
                            rect: ['1049px', '169px', '35px', '35px', 'auto', 'auto'],
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                        },
                        {
                            id: 'Sol_ex',
                            type: 'image',
                            rect: ['349px', '437px', '984px', '515px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"S1.PNG",'0px','0px']
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
                            "eid447",
                            "top",
                            0,
                            0,
                            "linear",
                            "${full_screen2}",
                            '169px',
                            '169px'
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
                            "eid448",
                            "left",
                            0,
                            0,
                            "linear",
                            "${full_screen2}",
                            '1049px',
                            '1049px'
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
                            rect: ['-551px', '212px', '1920', '1080', 'auto', 'auto'],
                            source: ['media/demo2%285%29.webm'],
                            id: 'demo1',
                            type: 'video',
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
                            "top",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '212px',
                            '-8px'
                        ],
                        [
                            "eid475",
                            "scaleY",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '0.55',
                            '1'
                        ],
                        [
                            "eid476",
                            "clip",
                            0,
                            431,
                            "linear",
                            "${demo1}",
                            [160.5763397216797,1644.1224365234375,921.5115966796875,287.963623046875],
                            [155.8602752685547,1646.6009521484375,928.0403442382812,281.6759033203125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid477",
                            "clip",
                            431,
                            145,
                            "linear",
                            "${demo1}",
                            [155.8602752685547,1646.6009521484375,928.0403442382812,281.6759033203125],
                            [154.26731872558594,1647.0155029296875,930.2455444335938,287.1178283691406],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid478",
                            "clip",
                            576,
                            116,
                            "linear",
                            "${demo1}",
                            [154.26731872558594,1647.0155029296875,930.2455444335938,287.1178283691406],
                            [153,1646.3194580078125,932,285.0641174316406],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid479",
                            "clip",
                            692,
                            58,
                            "linear",
                            "${demo1}",
                            [153,1646.3194580078125,932,285.0641174316406],
                            [144.56053161621094,1646.2034912109375,937.6234741210938,282.29791259765625],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid480",
                            "clip",
                            750,
                            93,
                            "linear",
                            "${demo1}",
                            [144.56053161621094,1646.2034912109375,937.6234741210938,282.29791259765625],
                            [131.0282745361328,1649.0745849609375,946.640380859375,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid481",
                            "clip",
                            843,
                            657,
                            "linear",
                            "${demo1}",
                            [131.0282745361328,1649.0745849609375,946.640380859375,277.8624267578125],
                            [112.20880126953125,1633.2886962890625,972.684326171875,284.6044921875],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid483",
                            "scaleX",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '0.55',
                            '1.03'
                        ],
                        [
                            "eid485",
                            "width",
                            0,
                            0,
                            "linear",
                            "${demo1}",
                            '1930px',
                            '1930px'
                        ],
                        [
                            "eid482",
                            "height",
                            0,
                            0,
                            "linear",
                            "${demo1}",
                            '1080px',
                            '1080px'
                        ],
                        [
                            "eid484",
                            "left",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '-551px',
                            '-227px'
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
                            type: 'group',
                            id: 'IntroCopy',
                            rect: ['51px', '6px', '1019', '177px', 'auto', 'auto'],
                            transform: [[], [], [], ['0.7', '0.7']],
                            c: [
                            {
                                rect: ['75px', '22px', '962px', '462px', 'auto', 'auto'],
                                id: 'CASE_DEFINITION_BLUECopy',
                                type: 'image',
                                fill: ['rgba(0,0,0,0)', 'images/CASE_DEFINITION_BLUE.png', '0px', '0px']
                            },
                            {
                                font: ['Tahoma, Geneva, sans-serif', [34, 'px'], 'rgba(0,0,0,1.00)', '500', 'none', 'normal', 'break-word', 'normal'],
                                type: 'text',
                                align: 'left',
                                id: 'Text5Copy',
                                textStyle: ['', '', '', '', 'none'],
                                text: '<p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\">​<span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px;\">​</span></p><p style=\"margin: 0px; line-height: 39px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">​</span>Program your Gardner to<span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\"></span><span style=\"font-weight: 400; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">:</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">- Flash in &nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">Green</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\"> for 2 seconds</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">​- Make the sound </span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">\"Hello\"</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\"> then&nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">\"EV3\"</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">​- Flash in&nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">Orange</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\"> for 5 seconds</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">​-Make the sound&nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">\"Red\"</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-weight: 400; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">-Turns on</span><span style=\"font-weight: 700; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">&nbsp;Red</span><span style=\"font-weight: 400; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">&nbsp;for 3 seconds</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">​-Make the sound&nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">\"Stop \"</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">​-Flash in&nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">Orange</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\"> for 2 seconds&nbsp;</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">​-Make the sound &nbsp;</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">\"Start up\"</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400;\">&nbsp;</span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 39px;\"><span style=\"font-weight: 400; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">​-Reset the status light of your gardner</span></p><p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 32px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 41px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; line-height: 41px; font-family: Tahoma, Geneva, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">​</span></p><p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 500; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: 41px;\">&nbsp;</p><p style=\"margin:0px\">​</p>',
                                rect: ['176px', '-35px', '816px', '146px', 'auto', 'auto']
                            },
                            {
                                rect: ['-48px', '34px', '160px', '160px', 'auto', 'auto'],
                                id: 'ROBOT07Copy',
                                type: 'image',
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
            "TITRE_1_2": {
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
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6Copy2',
                            rect: ['-39px', '0px', '872px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 789.948486328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Exercise</span></p>'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            id: 'Text6',
                            rect: ['273px', '0px', '699px', '57px', 'auto', 'auto'],
                            clip: 'rect(0px 661.60986328125px 57px 0px)',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Digital tech\'; font-size: 62px; font-weight: 500;\">exercise</span></p>'
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
                            [0,661.60986328125,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0190_Ex1_edgeActions.js");
})("EDGE-6689876");
