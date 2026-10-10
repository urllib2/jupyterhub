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
                scaleToFit: "height",
                centerStage: "horizontal",
                resizeInstances: false,
                content: {
                    dom: [
                        {
                            id: 'intergace',
                            type: 'image',
                            rect: ['80px', '0px', '1450px', '1080px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"intergace.png",'0px','0px']
                        },
                        {
                            id: 'intro',
                            type: 'group',
                            rect: ['233px', '-38px', '1014', '318', 'auto', 'auto'],
                            c: [
                            {
                                id: 'CASE_DEFINITION_BLUE',
                                type: 'image',
                                rect: ['165px', '157px', '882px', '142px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"CASE_DEFINITION_VIOLET1.png",'0px','0px']
                            },
                            {
                                id: 'ROBOT06',
                                type: 'image',
                                rect: ['18px', '139px', '147px', '160px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"ROBOTT.png",'0px','0px']
                            },
                            {
                                id: 'Definition',
                                type: 'text',
                                rect: ['209px', '181px', '803px', '112px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\">Programmer votre robot pour qu'il avance <span style=\"color: rgb(255, 87, 0);\">50 cm</span>.                                                   Saisissez vos tests dans le tableau suivant:</p><p style=\"margin:0px\"><span style=\"font-family: 'Trebuchet MS', Arial, Helvetica, sans-serif;\">​</span></p>",
                                font: ['Trebuchet MS, Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "", "break-word", "normal"]
                            }]
                        },
                        {
                            id: 'demonstration',
                            type: 'group',
                            rect: ['923px', '358px', '677', '381', 'auto', 'auto'],
                            c: [
                            {
                                id: 'video',
                                symbolName: 'video',
                                type: 'rect',
                                rect: ['-847px', '-369px', '1920', '1080', 'auto', 'auto'],
                                clip: 'rect(105.25579833984375px 1417.7603759765625px 1080px 51.41136169433594px)',
                                transform: [[],[],[],['1.01728','1.01111']]
                            },
                            {
                                id: 'cache_cadre',
                                type: 'image',
                                rect: ['-842px', '516px', '1449px', '202px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                            },
                            {
                                id: 'full_screen',
                                type: 'image',
                                rect: ['-274px', '495px', '50px', '50px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'demo',
                            type: 'group',
                            rect: ['-244px', '272px', '1051px', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1054.28564453125px 82px 900px)',
                            cursor: 'pointer',
                            transform: [[],[],[],['1.09756','1.09756']],
                            c: [
                            {
                                id: 'ORANGECopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREYCopy',
                                type: 'image',
                                rect: ['910px', '7px', '130px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text3Copy',
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
                                rect: ['945px', '0px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"play1.png",'0px','0px']
                            },
                            {
                                id: 'pause',
                                type: 'image',
                                rect: ['945px', '0px', '70px', '70px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"pause.png",'0px','0px']
                            }]
                        },
                        {
                            id: 'Group',
                            type: 'group',
                            rect: ['723px', '482px', '986px', '367', 'auto', 'auto'],
                            c: [
                            {
                                id: 'ligne_1',
                                type: 'group',
                                rect: ['0px', '44px', '773', '82', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'pack22',
                                    type: 'image',
                                    rect: ['0px', '0px', '773px', '82px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"pack2.png",'0px','0px']
                                },
                                {
                                    id: 'Text5',
                                    type: 'text',
                                    rect: ['541px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'Text1',
                                    type: 'text',
                                    rect: ['289px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'test',
                                    type: 'text',
                                    rect: ['35px', '3px', '210px', '72px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: MagistralC; font-size: 60px;\">Test 1</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'ligne_2',
                                type: 'group',
                                rect: ['0px', '123px', '773', '82', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'pack22Copy',
                                    type: 'image',
                                    rect: ['0px', '0px', '773px', '82px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"pack2.png",'0px','0px']
                                },
                                {
                                    id: 'Text6',
                                    type: 'text',
                                    rect: ['541px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'Text2',
                                    type: 'text',
                                    rect: ['289px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'testCopy',
                                    type: 'text',
                                    rect: ['35px', '3px', '210px', '72px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​<span style=\"font-family: MagistralC; font-size: 60px;\">Test 2</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'ligne_3',
                                type: 'group',
                                rect: ['0px', '205px', '773', '82', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'pack22Copy2',
                                    type: 'image',
                                    rect: ['0px', '0px', '773px', '82px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"pack2.png",'0px','0px']
                                },
                                {
                                    id: 'Text7',
                                    type: 'text',
                                    rect: ['541px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'Text3',
                                    type: 'text',
                                    rect: ['289px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'testCopy2',
                                    type: 'text',
                                    rect: ['35px', '3px', '210px', '72px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\"><span style=\"font-family: MagistralC; font-size: 60px;\">Test 3</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'ligne_4',
                                type: 'group',
                                rect: ['0px', '285px', '773', '82', 'auto', 'auto'],
                                c: [
                                {
                                    id: 'pack22Copy3',
                                    type: 'image',
                                    rect: ['0px', '0px', '773px', '82px', 'auto', 'auto'],
                                    fill: ["rgba(0,0,0,0)",im+"pack2.png",'0px','0px']
                                },
                                {
                                    id: 'Text8',
                                    type: 'text',
                                    rect: ['541px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'Text4',
                                    type: 'text',
                                    rect: ['289px', '3px', '218px', '57px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\">​</p>",
                                    font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "normal"]
                                },
                                {
                                    id: 'testCopy3',
                                    type: 'text',
                                    rect: ['35px', '3px', '210px', '72px', 'auto', 'auto'],
                                    text: "<p style=\"margin: 0px;\"><span style=\"font-family: MagistralC; font-size: 60px;\">Test 4</span></p>",
                                    align: "left",
                                    font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                    textStyle: ["", "", "", "", "none"]
                                }]
                            },
                            {
                                id: 'testCopy4',
                                type: 'text',
                                rect: ['20px', '0px', '975px', '72px', 'auto', 'auto'],
                                text: "<p style=\"margin: 0px;\"><span style=\"font-family: 'Palatino Linotype', 'Book Antigua', Palatino, serif; font-size: 34px; color: rgb(40, 48, 129); font-weight: 900;\">&nbsp; &nbsp;N° de test &nbsp; &nbsp; &nbsp; &nbsp;Nbr Rotations &nbsp; &nbsp;Distance (cm)</span></p>",
                                align: "left",
                                font: ['Arial, Helvetica, sans-serif', [24, "px"], "rgba(0,0,0,1)", "400", "none", "normal", "break-word", "normal"],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'valider',
                            type: 'group',
                            rect: ['282px', '853px', '1040', '82', 'auto', 'auto'],
                            clip: 'rect(0px 1040px 82px 834.17724609375px)',
                            cursor: 'pointer',
                            c: [
                            {
                                id: 'ORANGE',
                                type: 'image',
                                rect: ['847px', '7px', '193px', '60px', 'auto', 'auto'],
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_ORANGE.png",'0px','0px']
                            },
                            {
                                id: 'GREY',
                                type: 'image',
                                rect: ['847px', '7px', '193px', '60px', 'auto', 'auto'],
                                cursor: 'pointer',
                                fill: ["rgba(0,0,0,0)",im+"BOUTON_STANDARD_GREY.png",'0px','0px']
                            },
                            {
                                id: 'Text311',
                                type: 'text',
                                rect: ['856px', '16px', '180px', '44px', 'auto', 'auto'],
                                cursor: 'pointer',
                                text: "<p style=\"margin: 0px;\">​ &nbsp;Enregistrer&nbsp;</p>",
                                align: "left",
                                font: ['MagistralC', [30, "px"], "rgba(0,0,0,1)", "600", "none", "normal", "break-word", ""],
                                textStyle: ["", "", "", "", "none"]
                            }]
                        },
                        {
                            id: 'message',
                            type: 'text',
                            rect: ['968px', '386px', '425px', '82px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            align: "left",
                            font: ['Palatino Linotype, Book Antigua, Palatino, serif', [39, "px"], "rgba(163,10,10,1.00)", "600", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_1',
                            type: 'rect',
                            rect: ['533px', '29', '390', '57', 'auto', 'auto']
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
                    duration: 5500,
                    autoPlay: false,
                    data: [

                    ]
                }
            },
            "TITRE_1": {
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
                            rect: ['-62px', '0px', '655px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: center; text-indent: 0px; line-height: normal; color: rgb(0, 61, 65); font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">​Test et observations</p>',
                            font: ['Digital-7', [146, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 657.021728515625px 57px 0px)',
                            id: 'Text6Copy2'
                        },
                        {
                            type: 'text',
                            align: 'center',
                            textStyle: ['', '', '', '', 'none'],
                            rect: ['-54px', '0px', '699px', '57px', 'auto', 'auto'],
                            text: '<p style=\"margin: 0px; font-style: normal; text-decoration: none; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal; font-family: \'Digital tech\'; font-size: 64px; font-weight: normal;\">Test et observations</p>',
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
                            [0,565.53173828125,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid201",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,565.53173828125,57,0],
                            [0,650.10595703125,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid203",
                            "clip",
                            4750,
                            0,
                            "linear",
                            "${Text6}",
                            [0,650.10595703125,57,0],
                            [0,650.10595703125,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
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
                            transform: [[], [], [], ['1.00506']],
                            source: ['media/50.webm'],
                            id: 'demo2',
                            preload: 'auto',
                            rect: ['-224px', '243px', '1920', '1080', 'auto', 'auto'],
                            type: 'video',
                            clip: 'rect(148.4701385498047px 1664.322998046875px 932.6868896484375px 277.8624267578125px)',
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
                            "eid332",
                            "scaleY",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '0.43',
                            '1'
                        ],
                        [
                            "eid333",
                            "clip",
                            0,
                            692,
                            "linear",
                            "${demo2}",
                            [148.4701385498047,1664.322998046875,932.6868896484375,277.8624267578125],
                            [153,1664.3229980469,932,277.86242675781],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid334",
                            "clip",
                            692,
                            151,
                            "linear",
                            "${demo2}",
                            [153,1664.3229980469,932,277.86242675781],
                            [131.0282745361328,1664.322998046875,946.640380859375,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid335",
                            "clip",
                            843,
                            657,
                            "linear",
                            "${demo2}",
                            [131.0282745361328,1664.322998046875,946.640380859375,277.8624267578125],
                            [121.06890869140625,1661.2529296875,987.326904296875,277.8624267578125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid331",
                            "top",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '243px',
                            '-13px'
                        ],
                        [
                            "eid338",
                            "left",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '-628px',
                            '-224px'
                        ],
                        [
                            "eid336",
                            "height",
                            1500,
                            0,
                            "linear",
                            "${demo2}",
                            '1080px',
                            '1080px'
                        ],
                        [
                            "eid337",
                            "scaleX",
                            0,
                            1500,
                            "linear",
                            "${demo2}",
                            '0.43',
                            '1.00506'
                        ],
                        [
                            "eid339",
                            "width",
                            1500,
                            0,
                            "linear",
                            "${demo2}",
                            '1930px',
                            '1930px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0210_Test_edgeActions.js");
})("EDGE-6689876");
