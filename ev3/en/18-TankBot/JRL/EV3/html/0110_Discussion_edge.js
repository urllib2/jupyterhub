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
                            id: 'larobotique3',
                            type: 'video',
                            tag: 'video',
                            rect: ['165px', '151px', '1280', '720', 'auto', 'auto'],
                            clip: 'rect(99.7033920288086px 1104.57470703125px 622.3921508789063px 174.11753845214844px)',
                            source: [vid+"Exp.webm"],
                            preload: 'auto'
                        },
                        {
                            id: 'cache_cadre',
                            type: 'image',
                            rect: ['80px', '878px', '1450px', '202px', 'auto', 'auto'],
                            fill: ["rgba(0,0,0,0)",im+"cache_cadre.png",'0px','0px']
                        },
                        {
                            id: 'full_screen',
                            type: 'image',
                            rect: ['1231px', '770px', '46px', '47px', 'auto', 'auto'],
                            clip: 'rect(0px 60px 60px 0px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"full_screen.png",'0px','-2.0799966720053px'],
                            transform: [[],[],[],['0.98039','0.96154']]
                        },
                        {
                            id: 'Text2',
                            type: 'text',
                            rect: ['190px', '138px', '1269px', '89px', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px; text-align: center;\"><span style=\"font-size: 44px;\">Find an explanation for the phenomenon seen in the following video ??​</span></p>",
                            align: "left",
                            font: ['\'Trebuchet MS\', Arial, Helvetica, sans-serif', [36, "px"], "rgba(0,0,0,1)", "500", "none", "normal", "break-word", "normal"],
                            textStyle: ["", "", "", "", "none"]
                        },
                        {
                            id: 'Text',
                            type: 'text',
                            rect: ['976', '65', 'auto', 'auto', 'auto', 'auto'],
                            text: "<p style=\"margin: 0px;\">​</p>",
                            font: ['Arial, Helvetica, sans-serif', [24, ""], "rgba(0,0,0,1)", "normal", "none", "", "break-word", "nowrap"]
                        },
                        {
                            id: 'play_boutton',
                            type: 'image',
                            rect: ['470px', '375px', '653px', '340px', 'auto', 'auto'],
                            clip: 'rect(5.5556640625px 496.05517578125px 340px 152.77734375px)',
                            cursor: 'pointer',
                            fill: ["rgba(0,0,0,0)",im+"play3.png",'0px','0px'],
                            transform: [[],[],[],['0.42265','0.43527']]
                        },
                        {
                            id: 'a7_discution',
                            type: 'image',
                            rect: ['31px', '205px', '1024px', '724px', 'auto', 'auto'],
                            clip: 'rect(84px 308px 654px 84px)',
                            fill: ["rgba(0,0,0,0)",im+"a7_discution.png",'0px','0px']
                        },
                        {
                            id: 'a7_discution2',
                            type: 'image',
                            rect: ['558px', '183px', '1024px', '724px', 'auto', 'auto'],
                            clip: 'rect(94px 950px 656px 734px)',
                            fill: ["rgba(0,0,0,0)",im+"a7_discution.png",'0px','0px']
                        },
                        {
                            id: 'TITRE',
                            symbolName: 'TITRE_2_1',
                            type: 'rect',
                            rect: ['816px', '30px', '390', '57', 'auto', 'auto']
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
                        [
                            "eid213",
                            "left",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '1231px',
                            '1432px'
                        ],
                        [
                            "eid211",
                            "height",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '52px',
                            '57px'
                        ],
                        [
                            "eid212",
                            "top",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '770px',
                            '815px'
                        ],
                        [
                            "eid214",
                            "width",
                            5000,
                            500,
                            "linear",
                            "${full_screen}",
                            '51px',
                            '56px'
                        ],
                        [
                            "eid351",
                            "scaleY",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '1.03456',
                            '1.49'
                        ],
                        [
                            "eid352",
                            "scaleX",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '1.03456',
                            '1.49'
                        ],
                        [
                            "eid335",
                            "scaleX",
                            5000,
                            0,
                            "linear",
                            "${full_screen}",
                            '0.98039',
                            '0.98039'
                        ],
                        [
                            "eid354",
                            "clip",
                            5000,
                            250,
                            "linear",
                            "${larobotique3}",
                            [99.7033920288086,1104.57470703125,622.3921508789062,174.11753845214844],
                            [84.48918151855469,1108.404296875,641.7362060546875,222.66876220703125],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid355",
                            "clip",
                            5250,
                            250,
                            "linear",
                            "${larobotique3}",
                            [84.48918151855469,1108.404296875,641.7362060546875,222.66876220703125],
                            [77.93338775634766,1106.4544677734375,659.1227416992188,174.11753845214844],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid350",
                            "top",
                            5000,
                            500,
                            "linear",
                            "${larobotique3}",
                            '195px',
                            '151px'
                        ],
                        [
                            "eid336",
                            "scaleY",
                            5000,
                            0,
                            "linear",
                            "${full_screen}",
                            '0.96154',
                            '0.96154'
                        ],
                        [
                            "eid356",
                            "left",
                            5000,
                            0,
                            "linear",
                            "${larobotique3}",
                            '165px',
                            '165px'
                        ],
                        [
                            "eid357",
                            "left",
                            5500,
                            0,
                            "linear",
                            "${larobotique3}",
                            '165px',
                            '165px'
                        ]
                    ]
                }
            },
            "TITRE_2_1": {
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
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\"><span style=\"color: rgb(0, 61, 65); font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">Discussion</span></p>',
                            rect: ['-166px', '0px', '859px', '57px', 'auto', 'auto'],
                            align: 'center',
                            font: ['Digital-7', [117, 'px'], 'rgba(1,66,65,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 900px 57px -196.29614257813px)',
                            textStyle: ['', '', '', '', 'none']
                        },
                        {
                            type: 'text',
                            id: 'Text6',
                            text: '<p style=\"margin: 0px; font-family: Digital-7; font-weight: 600; font-style: normal; text-decoration: none; font-size: 62px; color: rgb(93, 216, 223); background-color: rgba(0, 0, 0, 0); letter-spacing: 0px; text-transform: none; word-spacing: 0px; text-align: left; text-indent: 0px; line-height: normal;\">​<span style=\"font-size: 62px; font-family: \'Digital tech\'; font-weight: 500;\">discussion</span></p>',
                            rect: ['-166px', '-1px', '766px', '57px', 'auto', 'auto'],
                            align: 'center',
                            font: ['Digital-7', [117, 'px'], 'rgba(152,252,250,1.00)', '700', 'none', 'normal', 'break-word', 'normal'],
                            clip: 'rect(0px 650px 57px -155.55541992188px)',
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
                    duration: 5000,
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
                            [0,515,57,0],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid167",
                            "clip",
                            4000,
                            250,
                            "linear",
                            "${Text6}",
                            [0,515,57,0],
                            [0,530,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid173",
                            "clip",
                            4250,
                            250,
                            "linear",
                            "${Text6}",
                            [0,530,57,-155.555419921875],
                            [0,555,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid175",
                            "clip",
                            4500,
                            250,
                            "linear",
                            "${Text6}",
                            [0,555,57,-155.555419921875],
                            [0,595,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid177",
                            "clip",
                            4750,
                            250,
                            "linear",
                            "${Text6}",
                            [0,595,57,-155.555419921875],
                            [0,650,57,-155.55541992188],
                            {valueTemplate: 'rect(@@0@@px @@1@@px @@2@@px @@3@@px)'}
                        ],
                        [
                            "eid476",
                            "top",
                            5000,
                            0,
                            "linear",
                            "${Text6}",
                            '-1px',
                            '-1px'
                        ]
                    ]
                }
            }
        };

    AdobeEdge.registerCompositionDefn(compId, symbols, fonts, scripts, resources, opts);

    if (!window.edge_authoring_mode) AdobeEdge.getComposition(compId).load("0110_Discussion_edgeActions.js");
})("EDGE-6689876");
