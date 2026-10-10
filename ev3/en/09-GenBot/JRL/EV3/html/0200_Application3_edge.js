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
                            rect: ['229px', '106px', '1019', '177', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['81px', '15px', '962px', '462px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_BLUE.png",'0px','0px']
                            },
                            {
                                id: 'Text5',
                                type: 'text',
                                rect: ['146px', '57px', '860px', '157px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">Write a program allowing to:</span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> -Emit the sound </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 700;\">\"One\", \"Two\", \"Three\"</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> and display at the same time </span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif; font-weight: 700;\">\"dot1\", \"dot2\", \"dot3\"</span><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\"> </span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- Go Forward 35.2 cm with a power = 20 </span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">-Stops 1 second while emitting the sound \"Speed Up\" </span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">- Go Forward 44 cm with a power = 40 </span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">-Emit the sound \"Good Job\" </span></p><p style=\"margin: 0px;\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">-Display an image of your choice for 5 seconds​</span></p>",
                                align: "left",
                                font: ['Tahoma, Geneva, sans-serif', [34, "px"], "rgba(0,0,0,1.00)", "400", "none", "normal", "break-word", "normal"],
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
                                    rect: ['178px', '443px', '111px', '90px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px; text-align: left;\"><span style=\"color: rgb(90, 191, 107);\">&nbsp; <span style=\"font-size: 50px;\">&nbsp;</span></span>:</p>",
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
                                rect: ['-856px', '-402px', '1920', '1080', 'auto', 'auto'],
                                transform: [[],[],[],['1.00111']]
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
                                rect: ['-115px', '456px', '48px', '48px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1_1',
                            type: 'rect',
                            rect: ['531px', '29', '390', '57', 'auto', 'auto']
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
                            rect: ['573px', '514px', '517px', '250px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"Yes2.png",'0px','0px']
                        },
                        {
                            id: 'solu',
                            type: 'group',
                            rect: ['-184px', '563px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 900px)',
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
                            '172px',
                            '172px'
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
                            '1016px',
                            '1016px'
                        ]
                    ]
                }
            },
            "hello": {
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
                            id: 'Text2',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['0px', '0px', '122px', '90px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px;\">​<span style=\"color: rgb(0, 0, 0);\">Hello</span></p>',
                            display: 'block',
                            font: ['Georgia, Times New Roman, Times, serif', [28, 'px'], 'rgba(255,255,255,1)', '600', 'none', 'normal', 'break-word', 'normal'],
                            align: 'left',
                            opacity: '0.84496111509412'
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            rect: [null, null, '122px', '90px']
                        }
                    }
                },
                timeline: {
                    duration: 1000,
                    autoPlay: true,
                    data: [
                        [
                            "eid222",
                            "opacity",
                            120,
                            130,
                            "linear",
                            "${Text2}",
                            '0.84496111509412',
                            '0.6124029755592346'
                        ],
                        [
                            "eid220",
                            "opacity",
                            250,
                            110,
                            "linear",
                            "${Text2}",
                            '0.6124029755592346',
                            '0.44186033990032'
                        ],
                        [
                            "eid221",
                            "opacity",
                            360,
                            75,
                            "linear",
                            "${Text2}",
                            '0.44185999035835266',
                            '0.27131735469944'
                        ],
                        [
                            "eid225",
                            "opacity",
                            555,
                            195,
                            "linear",
                            "${Text2}",
                            '0.2713170051574707',
                            '0.46659702016402'
                        ],
                        [
                            "eid226",
                            "opacity",
                            750,
                            105,
                            "linear",
                            "${Text2}",
                            '0.46659702016402',
                            '0.88442447481229'
                        ],
                        [
                            "eid228",
                            "opacity",
                            855,
                            145,
                            "linear",
                            "${Text2}",
                            '0.88442447481229',
                            '1'
                        ],
                        [
                            "eid229",
                            "font-size",
                            1000,
                            0,
                            "linear",
                            "${Text2}",
                            '28px',
                            '28px'
                        ],
                        [
                            "eid210",
                            "display",
                            500,
                            0,
                            "linear",
                            "${Text2}",
                            'block',
                            'none'
                        ],
                        [
                            "eid227",
                            "display",
                            595,
                            0,
                            "linear",
                            "${Text2}",
                            'none',
                            'block'
                        ],
                        [
                            "eid211",
                            "display",
                            1000,
                            0,
                            "linear",
                            "${Text2}",
                            'block',
                            'block'
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
                            transform: [[], [], [], ['1', '0.55564']],
                            source: ['media/ex3.webm'],
                            id: 'demo1',
                            preload: 'auto',
                            rect: ['-551px', '210px', '1920', '1080', 'auto', 'auto'],
                            type: 'video',
                            clip: 'rect(153.57232666015625px 1664.322998046875px 933.549560546875px 277.8624267578125px)',
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
                            "eid493",
                            "top",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '210px',
                            '-15px'
                        ],
                        [
                            "eid498",
                            "height",
                            1500,
                            0,
                            "linear",
                            "${demo1}",
                            '1080px',
                            '1080px'
                        ],
                        [
                            "eid495",
                            "clip",
                            0,
                            692,
                            "linear",
                            "${demo1}",
                            [153.57232666015625,1664.322998046875,933.549560546875,277.8624267578125],
                            [153,1664.3229980469,932,277.86242675781],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid496",
                            "clip",
                            692,
                            151,
                            "linear",
                            "${demo1}",
                            [153,1664.3229980469,932,277.86242675781],
                            [131.0282745361328,1664.322998046875,946.640380859375,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid497",
                            "clip",
                            843,
                            657,
                            "linear",
                            "${demo1}",
                            [131.0282745361328,1664.322998046875,946.640380859375,277.8624267578125],
                            [113.59771728515625,1667,986.5478515625,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid499",
                            "scaleX",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '0.55',
                            '1'
                        ],
                        [
                            "eid494",
                            "scaleY",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '0.55564',
                            '1'
                        ],
                        [
                            "eid501",
                            "width",
                            1500,
                            0,
                            "linear",
                            "${demo1}",
                            '1930px',
                            '1930px'
                        ],
                        [
                            "eid500",
                            "left",
                            0,
                            1500,
                            "linear",
                            "${demo1}",
                            '-551px',
                            '-239px'
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
                                id: 'CASE_DEFINITION_BLUECopy',
                                type: 'image',
                                rect: ['75px', '22px', '962px', '441px', 'auto', 'auto'],
                                fill: ['rgba(0,0,0,0)', 'images/CASE_DEFINITION_BLUE.png', '0px', '0px']
                            },
                            {
                                rect: ['168px', '58px', '816px', '157px', 'auto', 'auto'],
                                textStyle: ['', '', '', '', 'none'],
                                font: ['Tahoma, Geneva, sans-serif', [34, 'px'], 'rgba(0,0,0,1.00)', '500', 'none', 'normal', 'break-word', 'normal'],
                                id: 'Text5Copy',
                                text: '<p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">Write a program allowing to:</p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\"> -Emit the sound </span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">\"One\", \"Two\", \"Three\"</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\"> and display at the same time </span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 700;\">\"dot1\", \"dot2\", \"dot3\"</span><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\"> </span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">- Go Forward 35.2 cm with a power = 20 </span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">-Stops 1 second while emitting the sound \"Speed Up\" </span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">- Go Forward 44 cm with a power = 40 </span></p><p style=\"margin: 0px; font-family: Tahoma, Geneva, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif;\">-Emit the sound \"Good Job\" </span></p><p style=\"margin: 0px; font-family: \'Trebuchet MS\', Arial, Helvetica, sans-serif; font-weight: 400; font-style: normal; text-decoration: none; font-size: 34px; color: rgb(0, 0, 0); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">-Display an image of your choice for 5 seconds​</p><p></p>',
                                align: 'left',
                                type: 'text'
                            },
                            {
                                id: 'ROBOT07Copy',
                                type: 'image',
                                rect: ['-48px', '34px', '165px', '168px', 'auto', 'auto'],
                                fill: ['rgba(0,0,0,0)', 'images/RobotP.png', '0px', '0px']
                            }]
                        }
                    ],
                    style: {
                        '${symbolSelector}': {
                            isStage: 'true',
                            rect: [undefined, undefined, '1085px', '469px']
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
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-56px', '-3px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Challenge</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['114px', '-4px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"font-size: 64px; font-family: \'Digital tech\'; font-weight: 400;\">challenge</span></p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 519.83837890625px 57px -78.28271484375px)',
                            id: 'Text6'
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

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0200_Application3_edgeActions.js");
})("EDGE-6689876");
